//! rp2040js 1.3.3's own test suites, ported by hand (MIT; see NOTICE):
//! `src/instructions.spec.ts`, `src/rp2040.spec.ts` and `src/sio.spec.ts`
//! at tag v1.3.3 (commit 7701ee0). Each test keeps the upstream title; the
//! encoders are the upstream `src/utils/assembler.ts` ones, argument
//! conventions included (byte offsets where upstream takes them).
//!
//! Upstream drives the core through a test driver whose `singleStep` is
//! `RP2040.step()` (one `executeInstruction`, no clock), whose
//! `setRegisters` writes `core.registers` and the flags, and whose
//! `writeUint*` go through the bus; the port's equivalents are below.
//! Where upstream writes a negative JavaScript number straight into a
//! register model that firmware can only reach through the 32-bit bus, the
//! port gets the bus's unsigned value; those tests say so.

use super::bus::RAM_START;
use super::*;

const R0: u32 = 0;
const R1: u32 = 1;
const R2: u32 = 2;
const R3: u32 = 3;
const R4: u32 = 4;
const R5: u32 = 5;
const R6: u32 = 6;
const R7: u32 = 7;
const R8: u32 = 8;
const R11: u32 = 11;
const R12: u32 = 12;
const IP: u32 = 12;
const SP: u32 = 13;
const LR: u32 = 14;
const PC: u32 = 15;
const VTOR: u32 = 0xe000_ed08;
const NVIC_ISPR: u32 = 0xe000_e200;
const NVIC_ICPR: u32 = 0xe000_e280;
const NVIC_ISER: u32 = 0xe000_e100;
const NVIC_ICER: u32 = 0xe000_e180;

// ---- src/utils/assembler.ts -----------------------------------------------------

mod asm {
    pub fn adcs(rdn: u32, rm: u32) -> u32 { (0b0100000101 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn adds1(rd: u32, rn: u32, imm3: u32) -> u32 { (0b0001110 << 9) | ((imm3 & 7) << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn adds2(rdn: u32, imm8: u32) -> u32 { (0b00110 << 11) | ((rdn & 7) << 8) | (imm8 & 0xff) }
    pub fn add_sp_plus_imm(rd: u32, imm8: u32) -> u32 { (0b10101 << 11) | ((rd & 7) << 8) | ((imm8 >> 2) & 0xff) }
    pub fn add_sp2(imm: u32) -> u32 { (0b101100000 << 7) | ((imm >> 2) & 0x7f) }
    pub fn adds_reg(rd: u32, rn: u32, rm: u32) -> u32 { (0b0001100 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn add_reg(rdn: u32, rm: u32) -> u32 { (0b01000100 << 8) | ((rdn & 8) << 4) | ((rm & 0xf) << 3) | (rdn & 7) }
    pub fn adr(rd: u32, imm8: u32) -> u32 { (0b10100 << 11) | ((rd & 7) << 8) | ((imm8 >> 2) & 0xff) }
    pub fn ands(rn: u32, rm: u32) -> u32 { (0b0100000000 << 6) | ((rm & 7) << 3) | (rn & 7) }
    pub fn asrs(rd: u32, rm: u32, imm5: u32) -> u32 { (0b00010 << 11) | ((imm5 & 0x1f) << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn asrs_reg(rdn: u32, rm: u32) -> u32 { (0b0100000100 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn bt1(cond: u32, imm8: i32) -> u32 { (0b1101 << 12) | ((cond & 0xf) << 8) | ((imm8 >> 1) as u32 & 0x1ff) }
    pub fn bt2(imm11: u32) -> u32 { (0b11100 << 11) | ((imm11 >> 1) & 0x7ff) }
    pub fn bics(rdn: u32, rm: u32) -> u32 { (0b0100001110 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn bl(imm: i32) -> u32 {
        let imm11 = ((imm >> 1) & 0x7ff) as u32;
        let imm10 = ((imm >> 12) & 0x3ff) as u32;
        let s = (imm < 0) as u32;
        let j2 = 1 - (((imm >> 22) & 1) as u32 ^ s);
        let j1 = 1 - (((imm >> 23) & 1) as u32 ^ s);
        (0b1101 << 28) | (j1 << 29) | (j2 << 27) | (imm11 << 16) | (0b11110 << 11) | (s << 10) | imm10
    }
    pub fn blx(rm: u32) -> u32 { (0b010001111 << 7) | (rm << 3) }
    pub fn bx(rm: u32) -> u32 { (0b010001110 << 7) | (rm << 3) }
    pub fn cmn(rn: u32, rm: u32) -> u32 { (0b0100001011 << 6) | ((rm & 7) << 3) | (rn & 7) }
    pub fn cmp_imm(rn: u32, imm8: u32) -> u32 { (0b00101 << 11) | ((rn & 7) << 8) | (imm8 & 0xff) }
    pub fn cmp_reg_t1(rn: u32, rm: u32) -> u32 { (0b0100001010 << 6) | ((rm & 7) << 3) | (rn & 7) }
    pub fn cmp_reg_t2(rn: u32, rm: u32) -> u32 { (0b01000101 << 8) | (((rn >> 3) & 1) << 7) | ((rm & 0xf) << 3) | (rn & 7) }
    pub const DMBSY: u32 = 0x8f50_f3bf;
    pub const DSBSY: u32 = 0x8f4f_f3bf;
    pub const ISBSY: u32 = 0x8f6f_f3bf;
    pub fn eors(rdn: u32, rm: u32) -> u32 { (0b0100000001 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn ldmia(rn: u32, regs: u32) -> u32 { (0b11001 << 11) | ((rn & 7) << 8) | (regs & 0xff) }
    pub fn ldr_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101100 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldr_imm(rt: u32, rn: u32, imm5: u32) -> u32 { (0b01101 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldr_lit(rt: u32, imm8: u32) -> u32 { (0b01001 << 11) | ((imm8 >> 2) & 0xff) | ((rt & 7) << 8) }
    pub fn ldrb(rt: u32, rn: u32, imm5: u32) -> u32 { (0b01111 << 11) | ((imm5 & 0x1f) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldr_sp(rt: u32, imm8: u32) -> u32 { (0b10011 << 11) | ((rt & 7) << 8) | ((imm8 >> 2) & 0xff) }
    pub fn ldrb_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101110 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldrh(rt: u32, rn: u32, imm5: u32) -> u32 { (0b10001 << 11) | (((imm5 >> 1) & 0xf) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldrh_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101101 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldrsb(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101011 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn ldrsh(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101111 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn lsls_reg(rdn: u32, rm: u32) -> u32 { (0b0100000010 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn lsls_imm(rd: u32, rm: u32, imm5: u32) -> u32 { ((imm5 & 0x1f) << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn lsrs(rd: u32, rm: u32, imm5: u32) -> u32 { (0b00001 << 11) | ((imm5 & 0x1f) << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn lsrs_reg(rdn: u32, rm: u32) -> u32 { (0b0100000011 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn mov(rd: u32, rm: u32) -> u32 { (0b01000110 << 8) | (((rd & 8 != 0) as u32) << 7) | (rm << 3) | (rd & 7) }
    pub fn movs(rd: u32, imm8: u32) -> u32 { (0b00100 << 11) | ((rd & 7) << 8) | (imm8 & 0xff) }
    pub fn movs_reg(rd: u32, rm: u32) -> u32 { ((rm & 7) << 3) | (rd & 7) }
    pub fn mrs(rd: u32, spec: u32) -> u32 { (0b1000 << 28) | ((rd & 0xf) << 24) | ((spec & 0xff) << 16) | 0b1111001111101111 }
    pub fn msr(spec: u32, rn: u32) -> u32 { (0b10001000 << 24) | ((spec & 0xff) << 16) | (0b111100111000 << 4) | (rn & 0xf) }
    pub fn muls(rn: u32, rdm: u32) -> u32 { (0b0100001101 << 6) | ((rn & 7) << 3) | (rdm & 7) }
    pub fn mvns(rd: u32, rm: u32) -> u32 { (0b0100001111 << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub const NOP: u32 = 0b1011111100000000;
    pub fn orrs(rn: u32, rm: u32) -> u32 { (0b0100001100 << 6) | ((rm & 7) << 3) | (rn & 7) }
    pub fn pop(p: bool, list: u32) -> u32 { (0b1011110 << 9) | ((p as u32) << 8) | list }
    pub fn push(m: bool, list: u32) -> u32 { (0b1011010 << 9) | ((m as u32) << 8) | list }
    pub fn rev(rd: u32, rn: u32) -> u32 { (0b1011101000 << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn rev16(rd: u32, rn: u32) -> u32 { (0b1011101001 << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn revsh(rd: u32, rn: u32) -> u32 { (0b1011101011 << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn ror(rdn: u32, rm: u32) -> u32 { (0b0100000111 << 6) | ((rm & 7) << 3) | (rdn & 7) }
    pub fn rsbs(rd: u32, rn: u32) -> u32 { (0b0100001001 << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn sbcs(rn: u32, rm: u32) -> u32 { (0b0100000110 << 6) | ((rm & 7) << 3) | (rn & 7) }
    pub fn stmia(rn: u32, regs: u32) -> u32 { (0b11000 << 11) | ((rn & 7) << 8) | (regs & 0xff) }
    pub fn str(rt: u32, rm: u32, imm5: u32) -> u32 { (0b01100 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((rm & 7) << 3) | (rt & 7) }
    pub fn str_sp(rt: u32, imm8: u32) -> u32 { (0b10010 << 11) | ((rt & 7) << 8) | ((imm8 >> 2) & 0xff) }
    pub fn str_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101000 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn strb(rt: u32, rm: u32, imm5: u32) -> u32 { (0b01110 << 11) | ((imm5 & 0x1f) << 6) | ((rm & 7) << 3) | (rt & 7) }
    pub fn strb_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101010 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn strh(rt: u32, rm: u32, imm5: u32) -> u32 { (0b10000 << 11) | (((imm5 >> 1) & 0x1f) << 6) | ((rm & 7) << 3) | (rt & 7) }
    pub fn strh_reg(rt: u32, rn: u32, rm: u32) -> u32 { (0b0101001 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rt & 7) }
    pub fn subs1(rd: u32, rn: u32, imm3: u32) -> u32 { (0b0001111 << 9) | ((imm3 & 7) << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn subs2(rdn: u32, imm8: u32) -> u32 { (0b00111 << 11) | ((rdn & 7) << 8) | (imm8 & 0xff) }
    pub fn subs_reg(rd: u32, rn: u32, rm: u32) -> u32 { (0b0001101 << 9) | ((rm & 7) << 6) | ((rn & 7) << 3) | (rd & 7) }
    pub fn sub_sp(imm: u32) -> u32 { (0b101100001 << 7) | ((imm >> 2) & 0x7f) }
    pub fn svc(imm8: u32) -> u32 { (0b11011111 << 8) | (imm8 & 0xff) }
    pub fn sxtb(rd: u32, rm: u32) -> u32 { (0b1011001001 << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn sxth(rd: u32, rm: u32) -> u32 { (0b1011001000 << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn tst(rm: u32, rn: u32) -> u32 { (0b0100001000 << 6) | ((rn & 7) << 3) | (rm & 7) }
    pub fn uxtb(rd: u32, rm: u32) -> u32 { (0b1011001011 << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub fn udf(imm8: u32) -> u32 { (0b11011110 << 8) | (imm8 & 0xff) }
    pub fn udf2(imm16: u32) -> u32 { (0b111101111111 << 4) | ((imm16 >> 12) & 0xf) | (0b1010 << 28) | ((imm16 & 0xfff) << 16) }
    pub fn uxth(rd: u32, rm: u32) -> u32 { (0b1011001010 << 6) | ((rm & 7) << 3) | (rd & 7) }
    pub const WFI: u32 = 0b1011111100110000;
    pub const YIELD: u32 = 0b1011111100010000;
}
use asm::*;

// ---- the test driver --------------------------------------------------------------

const RAM: u32 = RAM_START;

/// A fresh RP2040 with PC at the start of SRAM and `op` there (a 32-bit
/// value is written as a word, as upstream's `writeUint32(pc, opcode)`).
fn at(op: u32) -> Rp2040 {
    let mut m = Rp2040::new();
    m.cpu.r[15] = RAM;
    put(&mut m, RAM, op);
    m
}
fn put(m: &mut Rp2040, addr: u32, op: u32) {
    if op > 0xffff {
        m.write_u32(addr, op);
    } else {
        m.write_u16(addr, op);
    }
}
fn step(m: &mut Rp2040) {
    m.execute_instruction();
}
fn nzcv(m: &Rp2040) -> (bool, bool, bool, bool) {
    (m.cpu.n, m.cpu.z, m.cpu.c, m.cpu.v)
}
fn nz(m: &Rp2040) -> (bool, bool) {
    (m.cpu.n, m.cpu.z)
}
fn reg(m: &mut Rp2040, n: u32, v: u32) {
    if n == SP {
        m.cpu.set_sp(v);
    } else {
        m.cpu.r[n as usize] = v;
    }
}
fn r(m: &Rp2040, n: u32) -> u32 {
    m.cpu.r[n as usize]
}

/// One upstream single-instruction test: `op` at 0x20000000, registers and
/// carry set, one step.
fn exec(op: u32, regs: &[(u32, u32)], c: bool) -> Rp2040 {
    let mut m = at(op);
    for &(n, v) in regs {
        reg(&mut m, n, v);
    }
    m.cpu.c = c;
    step(&mut m);
    m
}

const F: bool = false;
const T: bool = true;

// ---- src/instructions.spec.ts ------------------------------------------------------

#[test]
fn adcs_r5_r4() {
    let m = exec(adcs(R5, R4), &[(R4, 55), (R5, 66)], T);
    assert_eq!(r(&m, R5), 122);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn adcs_r5_r4_sets_negative_overflow() {
    let m = exec(adcs(R5, R4), &[(R4, 0x7fff_ffff), (R5, 0)], T);
    assert_eq!(r(&m, R5), 0x8000_0000);
    assert_eq!(nzcv(&m), (T, F, F, T));
}

#[test]
fn adcs_r3_r2_zero_plus_zero_with_carry_no_overflow() {
    let mut m = at(adcs(R3, R2));
    m.cpu.c = true;
    m.cpu.z = true;
    step(&mut m);
    assert_eq!(r(&m, R3), 1);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn adcs_r0_r0_0x80000000_sets_z_c_v() {
    let m = exec(adcs(R0, R0), &[(R0, 0x8000_0000)], F);
    assert_eq!(r(&m, R0), 0);
    assert_eq!(nzcv(&m), (F, T, T, T));
}

#[test]
fn add_sp_0x10() {
    let m = exec(add_sp2(0x10), &[(SP, 0x1000_0040)], F);
    assert_eq!(m.cpu.sp(), 0x1000_0050);
}

#[test]
fn add_r1_sp_4() {
    let m = exec(add_sp_plus_imm(R1, 0x10), &[(SP, 0x54), (R1, 0)], F);
    assert_eq!((m.cpu.sp(), r(&m, R1)), (0x54, 0x64));
}

#[test]
fn adds_r1_r2_3() {
    let m = exec(adds1(R1, R2, 3), &[(R2, 2)], F);
    assert_eq!(r(&m, R1), 5);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn adds_r1_1() {
    let m = exec(adds2(R1, 1), &[(R1, 0xffff_ffff)], F);
    assert_eq!(r(&m, R1), 0);
    assert_eq!(nzcv(&m), (F, T, T, F));
}

#[test]
fn adds_r1_r2_r7() {
    let m = exec(adds_reg(R1, R2, R7), &[(R2, 2), (R7, 27)], F);
    assert_eq!(r(&m, R1), 29);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn adds_r4_r4_r2() {
    let m = exec(adds_reg(R4, R4, R2), &[(R2, 0x74bc_8000), (R4, 0x4374_0000)], F);
    assert_eq!(r(&m, R4), 0xb830_8000);
    assert_eq!(nzcv(&m), (T, F, F, T));
}

#[test]
fn adds_r1_r1_r1() {
    let m = exec(adds_reg(R1, R1, R1), &[(R1, 0xbf8d_1424)], T);
    assert_eq!(r(&m, R1), 0x7f1a_2848);
    assert_eq!(nzcv(&m), (F, F, T, T));
}

#[test]
fn add_r1_ip() {
    let m = exec(add_reg(R1, IP), &[(R1, 66), (R12, 44)], F);
    assert_eq!(r(&m, R1), 110);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn add_r3_r12_t2_does_not_update_flags() {
    let m = exec(add_reg(R3, R12), &[(R3, 0x2000), (R12, 0xffff_e000)], F);
    assert_eq!(r(&m, R3), 0);
    assert_eq!(nzcv(&m), (F, F, F, F));
}

#[test]
fn add_sp_r8_does_not_update_flags() {
    let mut m = at(add_reg(SP, R8));
    reg(&mut m, SP, 0x2003_0000);
    reg(&mut m, R8, 0x13);
    m.cpu.z = true;
    step(&mut m);
    assert_eq!(m.cpu.sp(), 0x2003_0010);
    assert!(m.cpu.z);
}

#[test]
fn add_pc_r8() {
    let m = exec(add_reg(PC, R8), &[(R8, 0x11)], F);
    assert_eq!(m.cpu.pc(), 0x2000_0014);
}

#[test]
fn add_r7_pc_word_aligned() {
    let m = exec(add_reg(R7, PC), &[(R7, 0)], F);
    assert_eq!(r(&m, R7), 0x2000_0004);
}

#[test]
fn add_r7_pc_halfword_aligned() {
    let mut m = at(NOP);
    put(&mut m, RAM + 2, add_reg(R7, PC));
    step(&mut m);
    step(&mut m);
    assert_eq!(r(&m, R7), 0x2000_0006);
}

#[test]
fn adr_r4_0x50() {
    let m = exec(adr(R4, 0x50), &[], F);
    assert_eq!(r(&m, R4), 0x2000_0054);
}

#[test]
fn ands_r5_r0() {
    let m = exec(ands(R5, R0), &[(R5, 0xffff_0000), (R0, 0xf00f_ffff)], F);
    assert_eq!(r(&m, R5), 0xf00f_0000);
    assert_eq!(nz(&m), (T, F));
}

#[test]
fn asrs_r3_r2_31() {
    let m = exec(asrs(R3, R2, 31), &[(R2, 0x8000_0000)], T);
    assert_eq!((r(&m, R3), m.cpu.pc()), (0xffff_ffff, 0x2000_0002));
    assert_eq!((m.cpu.n, m.cpu.z, m.cpu.c), (T, F, F));
}

#[test]
fn asrs_r3_r2_32_updates_carry() {
    let m = exec(asrs(R3, R2, 0), &[(R2, 0x8000_0000)], F);
    assert_eq!((r(&m, R3), m.cpu.pc()), (0xffff_ffff, 0x2000_0002));
    assert_eq!((m.cpu.n, m.cpu.z, m.cpu.c), (T, F, T));
}

#[test]
fn asrs_r3_r4_variants() {
    // (r3, r4, carry in) -> (r3, N, Z, C), the five upstream cases in order.
    for (r3, r4, c_in, want, n, z, c) in [
        (0x8000_0040u32, 0xff50_0007u32, F, 0xff00_0000u32, T, F, T),
        (0x4000_0040, 50, T, 0, F, T, F),
        (0x4000_0040, 31, T, 0, F, T, T),
        (0x8000_0040, 50, T, 0xffff_ffff, T, F, T),
        (0x8000_0040, 0, T, 0x8000_0040, T, F, T),
    ] {
        let m = exec(asrs_reg(R3, R4), &[(R3, r3), (R4, r4)], c_in);
        assert_eq!((r(&m, R3), m.cpu.pc()), (want, 0x2000_0002), "{r3:#x} >> {r4}");
        assert_eq!((m.cpu.n, m.cpu.z, m.cpu.c), (n, z, c), "{r3:#x} >> {r4}");
    }
}

#[test]
fn bics_r0_r3() {
    let m = exec(bics(R0, R3), &[(R0, 0xff), (R3, 0x0f)], F);
    assert_eq!(r(&m, R0), 0xf0);
    assert_eq!(nz(&m), (F, F));
    let m = exec(bics(R0, R3), &[(R0, 0xffff_ffff), (R3, 0xffff)], F);
    assert_eq!(r(&m, R0), 0xffff_0000);
    assert_eq!(nz(&m), (T, F));
}

#[test]
fn bl_forward_backward() {
    for (off, pc) in [(0x34i32, 0x2000_0038u32), (-0x10, 0x2000_0004 - 0x10), (-3242, 0x2000_0004 - 3242)] {
        let m = exec(bl(off), &[], F);
        assert_eq!((m.cpu.pc(), r(&m, LR)), (pc, 0x2000_0005), "bl {off}");
    }
}

#[test]
fn blx_r3() {
    let m = exec(blx(R3), &[(R3, 0x2000_0201)], F);
    assert_eq!((m.cpu.pc(), r(&m, LR)), (0x2000_0200, 0x2000_0003));
}

#[test]
fn b_n_minus_20() {
    let mut m = Rp2040::new();
    m.cpu.r[15] = RAM + 9 * 2;
    put(&mut m, RAM + 9 * 2, bt2(0xfec));
    step(&mut m);
    assert_eq!(m.cpu.pc(), 0x2000_0002);
}

#[test]
fn bne_n_minus_6() {
    let mut m = Rp2040::new();
    m.cpu.r[15] = RAM + 9 * 2;
    m.cpu.z = false;
    put(&mut m, RAM + 9 * 2, bt1(1, 0x1f8));
    step(&mut m);
    assert_eq!(m.cpu.pc(), 0x2000_000e);
}

#[test]
fn bx_lr() {
    let m = exec(bx(LR), &[(LR, 0x1000_0200)], F);
    assert_eq!(m.cpu.pc(), 0x1000_0200);
}

#[test]
fn cmn_r7_r2() {
    let m = exec(cmn(R7, R2), &[(R2, 1), (R7, (-2i32) as u32)], F);
    assert_eq!((r(&m, R2), r(&m, R7)), (1, (-2i32) as u32));
    assert_eq!(nzcv(&m), (T, F, F, F));
}

#[test]
fn cmp_immediate_and_register_forms() {
    // (opcode, registers) -> NZCV, the upstream CMP cases in order.
    let cases: [(u32, &[(u32, u32)], (bool, bool, bool, bool)); 10] = [
        (cmp_imm(R5, 66), &[(R5, 60)], (T, F, F, F)),
        (cmp_imm(R0, 0), &[(R0, 0x8001_0133)], (T, F, T, F)),
        (cmp_reg_t1(R5, R0), &[(R5, 60), (R0, 56)], (F, F, T, F)),
        (cmp_reg_t1(R2, R0), &[(R0, 0xb71b_0000), (R2, 0x00b7_1b00)], (F, F, F, F)),
        (cmp_reg_t2(R11, R3), &[(R3, 8), (R11, 0xffff_ffff)], (T, F, T, F)),
        (cmp_reg_t2(IP, R6), &[(R6, 56), (R12, 60)], (F, F, T, F)),
        (cmp_reg_t2(R11, R3), &[(R3, 0), (R11, 0x8000_0000)], (T, F, T, F)),
        (cmp_reg_t1(R3, R7), &[(R3, 0), (R7, 0x8000_0000)], (T, F, F, T)),
        (cmp_reg_t2(R11, R3), &[(R3, 0x8000_0000), (R11, 0)], (T, F, F, T)),
        (cmp_reg_t1(R3, R7), &[(R3, 0x8000_0000), (R7, 0)], (T, F, T, F)),
    ];
    for (i, (op, regs, want)) in cases.iter().enumerate() {
        let m = exec(*op, regs, F);
        assert_eq!(nzcv(&m), *want, "case {i}");
    }
}

#[test]
fn barriers_are_32_bit() {
    for op in [DMBSY, DSBSY, ISBSY] {
        let m = exec(op, &[], F);
        assert_eq!(m.cpu.pc(), 0x2000_0004);
    }
}

#[test]
fn eors_r1_r3() {
    let m = exec(eors(R1, R3), &[(R1, 0xf0f0_f0f0), (R3, 0x08ff_3007)], F);
    assert_eq!(r(&m, R1), 0xf80f_c0f7);
    assert_eq!(nz(&m), (T, F));
}

#[test]
fn mov_r3_r8_and_r3_pc() {
    let m = exec(mov(R3, R8), &[(R8, 55)], F);
    assert_eq!(r(&m, R3), 55);
    let m = exec(mov(R3, PC), &[], F);
    assert_eq!(r(&m, R3), 0x2000_0004);
}

#[test]
fn muls_r0_r2() {
    for (r0, r2, want, n, z) in [
        (5u32, 1_000_000u32, 5_000_000u32, F, F),
        (2_654_435_769, 340_573_321, 1, F, F),
        (0, 1_000_000, 0, F, T),
        (u32::MAX, 1_000_000, (-1_000_000i32) as u32, T, F),
    ] {
        let m = exec(muls(R0, R2), &[(R0, r0), (R2, r2)], F);
        assert_eq!(r(&m, R2), want);
        if want != 1 {
            assert_eq!(nz(&m), (n, z));
        }
    }
}

#[test]
fn mvns_r4_r3() {
    let m = exec(mvns(R4, R3), &[(R3, 0x1111_5555)], F);
    assert_eq!(r(&m, R4), 0xeeee_aaaa);
    assert_eq!(nz(&m), (T, F));
}

#[test]
fn nop() {
    let m = exec(NOP, &[], F);
    assert_eq!(m.cpu.pc(), 0x2000_0002);
}

#[test]
fn orrs_r5_r0() {
    let m = exec(orrs(R5, R0), &[(R5, 0xf00f_0000), (R0, 0xf000_ffff)], F);
    assert_eq!(r(&m, R5), 0xf00f_ffff);
    assert_eq!(nz(&m), (T, F));
}

#[test]
fn pop_pc_r4_r5_r6() {
    let mut m = at(pop(true, (1 << R4) | (1 << R5) | (1 << R6)));
    reg(&mut m, SP, RAM + 0xf0);
    for (i, v) in [0x40, 0x50, 0x60, 0x42].iter().enumerate() {
        m.write_u32(0x2000_00f0 + 4 * i as u32, *v);
    }
    step(&mut m);
    assert_eq!(m.cpu.sp(), RAM + 0x100);
    assert_eq!((r(&m, R4), r(&m, R5), r(&m, R6), m.cpu.pc()), (0x40, 0x50, 0x60, 0x42));
}

#[test]
fn push_r4_r5_r6_lr() {
    let mut m = at(push(true, (1 << R4) | (1 << R5) | (1 << R6)));
    reg(&mut m, SP, RAM + 0x100);
    for (n, v) in [(R4, 0x40), (R5, 0x50), (R6, 0x60), (LR, 0x42)] {
        reg(&mut m, n, v);
    }
    step(&mut m);
    assert_eq!(m.cpu.sp(), RAM + 0xf0);
    for (i, v) in [0x40, 0x50, 0x60, 0x42].iter().enumerate() {
        assert_eq!(m.read_u8(0x2000_00f0 + 4 * i as u32), *v);
    }
}

#[test]
fn mrs_r0_ipsr() {
    let m = exec(mrs(R0, 5), &[(R0, 55)], F);
    assert_eq!((r(&m, R0), m.cpu.pc()), (0, 0x2000_0004));
}

#[test]
fn msr_msp_r0() {
    let m = exec(msr(8, R0), &[(R0, 0x1234)], F);
    assert_eq!((m.cpu.sp(), m.cpu.pc()), (0x1234, 0x2000_0004));
}

#[test]
fn movs_r5_128() {
    let m = exec(movs(R5, 128), &[], F);
    assert_eq!((r(&m, R5), m.cpu.pc()), (128, 0x2000_0002));
}

#[test]
fn ldmia_r0_r1_r2() {
    let mut m = at(ldmia(R0, (1 << R1) | (1 << R2)));
    reg(&mut m, R0, 0x2000_0010);
    m.write_u32(0x2000_0010, 0xf00d_f00d);
    m.write_u32(0x2000_0014, 0x4242);
    step(&mut m);
    assert_eq!((m.cpu.pc(), r(&m, R0), r(&m, R1), r(&m, R2)), (0x2000_0002, 0x2000_0018, 0xf00d_f00d, 0x4242));
}

#[test]
fn ldmia_r5_r5_no_writeback() {
    let mut m = at(ldmia(R5, 1 << R5));
    reg(&mut m, R5, 0x2000_0010);
    m.write_u32(0x2000_0010, 0xf00d_f00d);
    step(&mut m);
    assert_eq!((m.cpu.pc(), r(&m, R5)), (0x2000_0002, 0xf00d_f00d));
}

#[test]
fn ldr_forms() {
    let mut m = at(ldr_lit(R0, 148));
    m.write_u32(RAM + 152, 0x42);
    step(&mut m);
    assert_eq!((r(&m, R0), m.cpu.pc()), (0x42, 0x2000_0002));

    let mut m = at(ldr_imm(R3, R2, 24));
    reg(&mut m, R2, RAM);
    m.write_u32(RAM + 24, 0x55);
    step(&mut m);
    assert_eq!(r(&m, R3), 0x55);

    let mut m = at(ldr_sp(R3, 12));
    reg(&mut m, SP, RAM);
    m.write_u32(RAM + 12, 0x55);
    step(&mut m);
    assert_eq!(r(&m, R3), 0x55);

    let mut m = at(ldr_reg(R3, R5, R6));
    reg(&mut m, R5, RAM);
    reg(&mut m, R6, 8);
    m.write_u32(RAM + 8, 0xff55_4211);
    step(&mut m);
    assert_eq!(r(&m, R3), 0xff55_4211);
}

#[test]
fn ldrb_ldrh_forms() {
    let mut m = at(ldrb(R4, R2, 5));
    reg(&mut m, R2, RAM);
    m.write_u16(RAM + 5, 0x7766);
    step(&mut m);
    assert_eq!(r(&m, R4), 0x66);

    let mut m = at(ldrb_reg(R3, R5, R6));
    reg(&mut m, R5, RAM);
    reg(&mut m, R6, 8);
    m.write_u32(RAM + 8, 0xff55_4211);
    step(&mut m);
    assert_eq!(r(&m, R3), 0x11);

    for (imm, word, want) in [(4u32, 0xffff_7766u32, 0x7766u32), (6, 0x3344_7766, 0x3344)] {
        let mut m = at(ldrh(R3, R7, imm));
        reg(&mut m, R7, RAM);
        m.write_u32(RAM + 4, word);
        step(&mut m);
        assert_eq!(r(&m, R3), want);
    }

    let mut m = at(ldrh_reg(R3, R5, R6));
    reg(&mut m, R5, RAM);
    reg(&mut m, R6, 8);
    m.write_u32(RAM + 8, 0xff55_4211);
    step(&mut m);
    assert_eq!(r(&m, R3), 0x4211);
}

#[test]
fn ldrsb_ldrsh() {
    let mut m = at(ldrsb(R5, R3, R5));
    reg(&mut m, R3, RAM);
    reg(&mut m, R5, 6);
    m.write_u32(RAM + 6, 0x85);
    step(&mut m);
    assert_eq!(r(&m, R5), 0xffff_ff85);

    let mut m = at(ldrsh(R5, R3, R5));
    reg(&mut m, R3, RAM);
    reg(&mut m, R5, 6);
    m.write_u16(RAM + 6, 0xf055);
    step(&mut m);
    assert_eq!(r(&m, R5), 0xffff_f055);
}

#[test]
fn udf_1_and_udf_w_0_break() {
    // Upstream checks `onBreak` was called with the code; the port records
    // the break as its stop.
    let mut m = at(udf(1));
    m.step();
    assert_eq!((m.cpu.pc(), m.stop), (0x2000_0002, Some(Stop::Break { code: 1 })));
    let mut m = at(udf2(0));
    m.step();
    assert_eq!((m.cpu.pc(), m.stop), (0x2000_0004, Some(Stop::Break { code: 0 })));
}

#[test]
fn lsls_r5_r5_18_and_lsls_r5_r0() {
    let m = exec(lsls_imm(R5, R5, 18), &[(R5, 0b11)], F);
    assert_eq!((r(&m, R5), m.cpu.pc(), m.cpu.c), (0b11 << 18, 0x2000_0002, F));
    let m = exec(lsls_reg(R5, R0), &[(R5, 0b11), (R0, 0xff00_3302)], F);
    assert_eq!((r(&m, R5), m.cpu.pc(), m.cpu.c), (0b1100, 0x2000_0002, F));
}

#[test]
fn lsls_r3_r4_shift_32() {
    let m = exec(lsls_reg(R3, R4), &[(R3, 1), (R4, 0x20)], F);
    assert_eq!((r(&m, R3), m.cpu.pc()), (0, 0x2000_0002));
    assert_eq!((m.cpu.n, m.cpu.c, m.cpu.z), (F, T, T));
}

#[test]
fn lsls_r5_r5_18_with_carry() {
    let m = exec(lsls_imm(R5, R5, 18), &[(R5, 0x4001)], F);
    assert_eq!((r(&m, R5), m.cpu.c), (0x40000, T));
}

#[test]
fn lsrs_forms() {
    let m = exec(lsrs_reg(R5, R0), &[(R5, 0xff00_000f), (R0, 0xff00_3302)], F);
    assert_eq!((r(&m, R5), m.cpu.pc(), m.cpu.c), (0x3fc0_0003, 0x2000_0002, T));
    let m = exec(lsrs_reg(R2, R3), &[(R2, 10), (R3, 32)], F);
    assert_eq!((r(&m, R2), m.cpu.z, m.cpu.c), (0, T, F));
    let m = exec(lsrs(R1, R1, 1), &[(R1, 0b10)], F);
    assert_eq!((r(&m, R1), m.cpu.pc(), m.cpu.c), (1, 0x2000_0002, F));
    let m = exec(lsrs(R1, R1, 0), &[(R1, 0xffff_ffff)], F);
    assert_eq!((r(&m, R1), m.cpu.pc(), m.cpu.c), (0, 0x2000_0002, T));
}

#[test]
fn mov_sp_pc_keep_low_bits_clear_and_movs_reg() {
    let m = exec(mov(SP, R5), &[(R5, 0x53)], F);
    assert_eq!(m.cpu.sp(), 0x50);
    let m = exec(mov(PC, R5), &[(R5, 0x53)], F);
    assert_eq!(m.cpu.pc(), 0x52);
    let m = exec(movs_reg(R6, R5), &[(R5, 0x50)], F);
    assert_eq!(r(&m, R6), 0x50);
}

#[test]
fn rsbs_r0_r3() {
    let m = exec(rsbs(R0, R3), &[(R3, 100)], F);
    assert_eq!(r(&m, R0) as i32, -100);
    assert_eq!(nzcv(&m), (T, F, F, F));
    let m = exec(rsbs(R0, R3), &[(R3, 0)], F);
    assert_eq!(r(&m, R0), 0);
    assert_eq!(nzcv(&m), (F, T, T, F));
}

#[test]
fn rev_rev16_revsh() {
    let m = exec(rev(R2, R3), &[(R3, 0x1122_3344)], F);
    assert_eq!(r(&m, R2), 0x4433_2211);
    let m = exec(rev16(R0, R5), &[(R5, 0x1122_3344)], F);
    assert_eq!(r(&m, R0), 0x2211_4433);
    let m = exec(revsh(R1, R2), &[(R2, 0xeeaa_55f0)], F);
    assert_eq!(r(&m, R1), 0xffff_f055);
}

#[test]
fn ror_r5_r3_including_past_32() {
    for r3 in [0x2004u32, 0x2044] {
        let m = exec(ror(R5, R3), &[(R5, 0x1234_5678), (R3, r3)], F);
        assert_eq!((r(&m, R3), r(&m, R5)), (r3, 0x8123_4567));
        assert_eq!((m.cpu.n, m.cpu.z, m.cpu.c), (T, F, T));
    }
}

#[test]
fn sbcs_r0_r3() {
    let m = exec(sbcs(R0, R3), &[(R0, 100), (R3, 55)], F);
    assert_eq!(r(&m, R0), 44);
    assert_eq!(nzcv(&m), (F, F, T, F));
    let m = exec(sbcs(R0, R3), &[(R0, 0), (R3, 0xffff_ffff)], F);
    assert_eq!(r(&m, R0), 0);
    assert_eq!(nzcv(&m), (F, T, F, F));
}

#[test]
fn stmia_r0_r1_r2() {
    let m = exec(stmia(R0, (1 << R1) | (1 << R2)), &[(R0, 0x2000_0010), (R1, 0xf00d_f00d), (R2, 0x4242)], F);
    let mut m = m;
    assert_eq!((m.cpu.pc(), r(&m, R0)), (0x2000_0002, 0x2000_0018));
    assert_eq!((m.read_u32(0x2000_0010), m.read_u32(0x2000_0014)), (0xf00d_f00d, 0x4242));
}

#[test]
fn str_forms() {
    let mut m = exec(str(R6, R4, 20), &[(R4, RAM + 0x20), (R6, 0xf00d)], F);
    assert_eq!((m.read_u32(0x2000_0020 + 20), m.cpu.pc()), (0xf00d, 0x2000_0002));
    let mut m = exec(str_reg(R6, R4, R5), &[(R4, RAM + 0x20), (R5, 20), (R6, 0xf00d)], F);
    assert_eq!((m.read_u32(0x2000_0020 + 20), m.cpu.pc()), (0xf00d, 0x2000_0002));
    let mut m = exec(str_sp(R3, 12), &[(R3, 0xaa55), (SP, RAM)], F);
    assert_eq!((m.read_u8(RAM + 12), m.read_u8(RAM + 13)), (0x55, 0xaa));
    // r1 + r3 past 32 bits wraps.
    let mut m = exec(str_reg(R2, R1, R3), &[(R1, (-4i32) as u32), (R3, 0x2004_1e50), (R2, 0x420_1337)], F);
    assert_eq!((m.read_u32(0x2004_1e4c), m.cpu.pc()), (0x420_1337, 0x2000_0002));
}

#[test]
fn strb_strh_forms() {
    for (op, regs) in [
        (strb(R6, R4, 1), vec![(R4, RAM + 0x20), (R6, 0xf055)]),
        (strb_reg(R6, R4, R5), vec![(R4, RAM + 0x20), (R5, 1), (R6, 0xf055)]),
    ] {
        let mut m = at(op);
        m.write_u32(0x2000_0020, 0xf5f4_f3f2);
        for (n, v) in regs {
            reg(&mut m, n, v);
        }
        step(&mut m);
        assert_eq!((m.read_u32(0x2000_0020), m.cpu.pc()), (0xf5f4_55f2, 0x2000_0002));
    }
    for (op, regs) in [
        (strh(R6, R4, 2), vec![(R4, RAM + 0x20), (R6, 0x6655)]),
        (strh_reg(R6, R4, R1), vec![(R4, RAM + 0x20), (R1, 2), (R6, 0x6655)]),
    ] {
        let mut m = at(op);
        m.write_u32(0x2000_0020, 0xf5f4_f3f2);
        for (n, v) in regs {
            reg(&mut m, n, v);
        }
        step(&mut m);
        assert_eq!((m.read_u32(0x2000_0020), m.cpu.pc()), (0x6655_f3f2, 0x2000_0002));
    }
}

#[test]
fn sub_sp_0x10() {
    let m = exec(sub_sp(0x10), &[(SP, 0x1000_0040)], F);
    assert_eq!(m.cpu.sp(), 0x1000_0030);
}

#[test]
fn subs_forms() {
    let m = exec(subs2(R1, 1), &[(R1, 0x8000_0000)], F);
    assert_eq!(r(&m, R1), 0x7fff_ffff);
    assert_eq!(nzcv(&m), (F, F, T, T));
    let m = exec(subs1(R5, R3, 5), &[(R3, 0)], F);
    assert_eq!(r(&m, R5) as i32, -5);
    assert_eq!(nzcv(&m), (T, F, F, F));
    for (rd, r3, r2, want, flags) in [
        (R5, 6u32, 5u32, 1u32, (F, F, T, F)),
        (R3, 0xffff_ffff, 8, 0xffff_fff7, (T, F, T, F)),
        (R5, 0, 0x8000_0000, 0x8000_0000, (T, F, F, T)),
        (R5, 0x8000_0000, 0x8000_0000, 0, (F, T, T, F)),
    ] {
        let m = exec(subs_reg(rd, R3, R2), &[(R3, r3), (R2, r2)], F);
        assert_eq!(r(&m, rd), want);
        assert_eq!(nzcv(&m), flags);
    }
}

#[test]
fn svc_raises_svcall() {
    const SVCALL_HANDLER: u32 = 0x2000_2000;
    let mut m = Rp2040::new();
    reg(&mut m, SP, 0x2000_4000);
    m.cpu.r[15] = 0x2000_4000;
    m.write_u16(0x2000_4000, svc(10));
    reg(&mut m, R0, 0x44);
    m.write_u32(VTOR, 0x2004_0000);
    m.write_u32(0x2004_0000 + 11 * 4, SVCALL_HANDLER);
    m.write_u16(SVCALL_HANDLER, movs(R0, 0x55));
    step(&mut m);
    assert!(m.cpu.pending_svcall);
    step(&mut m);
    assert!(!m.cpu.pending_svcall);
    assert_eq!((m.cpu.pc(), r(&m, R0)), (SVCALL_HANDLER + 2, 0x55));
}

#[test]
fn extends_tst_and_hints() {
    assert_eq!(r(&exec(sxtb(R2, R2), &[(R2, 0x2244_6688)], F), R2), 0xffff_ff88);
    assert_eq!(r(&exec(sxtb(R2, R2), &[(R2, 0x1234_5678)], F), R2), 0x78);
    assert_eq!(r(&exec(sxth(R2, R5), &[(R5, 0x2244_8765)], F), R2), 0xffff_8765);
    assert!(exec(tst(R1, R3), &[(R1, 0xf000_0000), (R3, 0xf000_4000)], F).cpu.n);
    assert!(exec(tst(R1, R3), &[(R1, 0xf0), (R3, 0x0f)], F).cpu.z);
    assert_eq!(r(&exec(uxtb(R5, R3), &[(R3, 0x1234_5678)], F), R5), 0x78);
    assert_eq!(r(&exec(uxth(R3, R1), &[(R1, 0x1234_5678)], F), R3), 0x5678);
    assert_eq!(exec(WFI, &[], F).cpu.pc(), 0x2000_0002);
    assert_eq!(exec(YIELD, &[], F).cpu.pc(), 0x2000_0002);
}

// ---- src/rp2040.spec.ts ----------------------------------------------------------------

#[test]
fn initializes_pc_and_sp_from_the_bootrom_vector_table() {
    let mut m = Rp2040::new();
    m.load_bootrom(&[0x2004_1f00, 0xee]);
    assert_eq!((m.cpu.sp(), m.cpu.pc()), (0x2004_1f00, 0xee));
}

// Upstream's three IO-register-write tests plant a spy peripheral at table
// key 0x10. R1 halts on peripherals instead of running them, so the ports
// check the value the bus hands a real one (IO_BANK0 at 0x40014000) at the
// halt; the atomic case checks the decoded alias, since the XOR result needs
// the peripheral's own read.

#[test]
fn io_write_replicates_8_bit_values_four_times() {
    let mut m = Rp2040::new();
    m.write_u8(0x4001_4123, 0x534);
    assert_eq!(m.peripheral_write, Some(PeripheralWrite { offset: 0x120, atomic: 0, value: 0x3434_3434 }));
}

#[test]
fn io_write_replicates_16_bit_values_twice() {
    let mut m = Rp2040::new();
    m.write_u16(0x4001_4123, 0x1234_5678);
    assert_eq!(m.peripheral_write, Some(PeripheralWrite { offset: 0x120, atomic: 0, value: 0x5678_5678 }));
}

#[test]
fn io_write_supports_atomic_register_aliases() {
    let mut m = Rp2040::new();
    m.write_u32(0x4001_5120, 0x0f);
    assert_eq!(m.peripheral_write, Some(PeripheralWrite { offset: 0x120, atomic: 1, value: 0x0f }));
}

/// The exception tests run from flash, which the bus can write.
fn exception_machine() -> Rp2040 {
    let mut m = Rp2040::new();
    reg(&mut m, SP, 0x2000_4000);
    m.cpu.r[15] = 0x1000_4001;
    m
}

#[test]
fn executes_an_exception_handler_and_returns() {
    const INT1_HANDLER: u32 = 0x1000_0100;
    let mut m = exception_machine();
    m.cpu.r[0] = 0x44;
    m.cpu.pending_interrupts = 1 << 1;
    m.cpu.enabled_interrupts = 1 << 1;
    m.cpu.interrupts_updated = true;
    m.write_u32(VTOR, 0x1000_0000);
    m.write_u32(0x1000_0000 + 17 * 4, INT1_HANDLER);
    m.write_u16(INT1_HANDLER, movs(R0, 0x55));
    m.write_u16(INT1_HANDLER + 2, bx(LR));
    step(&mut m);
    assert_eq!((m.cpu.ipsr, m.cpu.pc(), r(&m, R0)), (17, INT1_HANDLER + 2, 0x55));
    step(&mut m);
    assert_eq!((m.cpu.pc(), r(&m, R0), m.cpu.ipsr), (0x1000_4000, 0x44, 0));
}

#[test]
fn returns_from_an_exception_with_pop_pc() {
    const INT1_HANDLER: u32 = 0x1000_0100;
    let mut m = exception_machine();
    m.cpu.r[4] = 105;
    m.cpu.pending_interrupts = 1 << 1;
    m.cpu.enabled_interrupts = 1 << 1;
    m.cpu.interrupts_updated = true;
    m.write_u32(VTOR, 0x1000_0000);
    m.write_u32(0x1000_0000 + 17 * 4, INT1_HANDLER);
    m.write_u16(INT1_HANDLER, push(true, 0b0111_0000));
    m.write_u16(INT1_HANDLER + 2, movs(R4, 42));
    m.write_u16(INT1_HANDLER + 4, pop(true, 0b0111_0000));
    step(&mut m);
    assert_eq!((m.cpu.ipsr, m.cpu.pc()), (17, INT1_HANDLER + 2));
    step(&mut m);
    assert_eq!(r(&m, R4), 42);
    step(&mut m);
    assert_eq!((m.cpu.pc(), r(&m, R4), m.cpu.ipsr), (0x1000_4000, 105, 0));
}

#[test]
fn exception_entry_clears_pending_for_user_irqs() {
    const INT31_HANDLER: u32 = 0x1000_3100;
    let mut m = exception_machine();
    m.write_u32(NVIC_ISPR, 1 << 31);
    m.cpu.enabled_interrupts = 1 << 31;
    m.cpu.interrupts_updated = true;
    m.write_u32(VTOR, 0x1000_0000);
    m.write_u32(0x1000_0000 + 47 * 4, INT31_HANDLER);
    m.write_u16(INT31_HANDLER, NOP);
    assert_eq!(m.cpu.pending_interrupts, 1 << 31);
    step(&mut m);
    assert_eq!(m.cpu.pending_interrupts, 0);
    assert_eq!(m.read_u32(NVIC_ISPR), 0);
}

#[test]
fn nvic_set_and_clear_registers() {
    let mut m = Rp2040::new();
    m.cpu.pending_interrupts = 0x1;
    m.write_u32(NVIC_ISPR, 0x10);
    assert_eq!(m.cpu.pending_interrupts, 0x11);

    let mut m = Rp2040::new();
    m.cpu.pending_interrupts = 0xff00_000f;
    m.write_u32(NVIC_ICPR, 0x1000_000f);
    // Only the high bits are cleared (upstream commit 5bc96994).
    assert_eq!(m.read_u32(NVIC_ISPR), 0xef00_000f);

    let mut m = Rp2040::new();
    m.cpu.enabled_interrupts = 0x1;
    m.write_u32(NVIC_ISER, 0x10);
    assert_eq!(m.cpu.enabled_interrupts, 0x11);

    let mut m = Rp2040::new();
    m.cpu.enabled_interrupts = 0xff;
    m.write_u32(NVIC_ICER, 0x10);
    assert_eq!(m.cpu.enabled_interrupts, 0xef);

    let mut m = Rp2040::new();
    m.cpu.enabled_interrupts = 0x1;
    assert_eq!((m.read_u32(NVIC_ISER), m.read_u32(NVIC_ICER)), (1, 1));
    m.cpu.pending_interrupts = 0x2;
    assert_eq!((m.read_u32(NVIC_ISPR), m.read_u32(NVIC_ICPR)), (2, 2));
}

#[test]
fn nvic_ipr_write_and_read() {
    let mut m = Rp2040::new();
    m.write_u32(0xe000_e40c, 0x0080_0000); // IRQ 14 -> priority 2
    let p = m.cpu.interrupt_priorities;
    assert_eq!(p, [!(1u32 << 14), 0, 1 << 14, 0]);
    assert_eq!(m.read_u32(0xe000_e40c), 0x0080_0000);

    let mut m = Rp2040::new();
    m.cpu.interrupt_priorities = [0, 0x001f_ffff, 0x0020_0000, 0xffc0_0000];
    assert_eq!(m.read_u32(0xe000_e414), 0xc0c0_8040);
}

// ---- src/sio.spec.ts ------------------------------------------------------------------

const SIO_DIV_UDIVIDEND: u32 = 0xd000_0060;
const SIO_DIV_UDIVISOR: u32 = 0xd000_0064;
const SIO_DIV_SDIVIDEND: u32 = 0xd000_0068;
const SIO_DIV_SDIVISOR: u32 = 0xd000_006c;
const SIO_DIV_QUOTIENT: u32 = 0xd000_0070;
const SIO_DIV_REMAINDER: u32 = 0xd000_0074;
const SIO_DIV_CSR: u32 = 0xd000_0078;
const SIO_SPINLOCK10: u32 = 0xd000_0128;
const SIO_SPINLOCKST: u32 = 0xd000_005c;

fn div(m: &mut Rp2040, dividend_reg: u32, a: i64, divisor_reg: u32, b: i64) {
    m.write_u32(dividend_reg, a as u32);
    m.write_u32(divisor_reg, b as u32);
}

#[test]
fn signed_divider_123456_by_minus_321() {
    let mut m = Rp2040::new();
    m.write_u32(SIO_DIV_SDIVIDEND, 123456);
    assert_eq!(m.read_u32(SIO_DIV_SDIVIDEND) as i32, 123456);
    m.write_u32(SIO_DIV_SDIVISOR, (-321i32) as u32);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 3);
    assert_eq!(m.read_u32(SIO_DIV_SDIVISOR) as i32, -321);
    assert_eq!(m.read_u32(SIO_DIV_REMAINDER) as i32, 192);
    assert_eq!(m.read_u32(SIO_DIV_QUOTIENT) as i32, -384);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 1);
}

#[test]
fn signed_divider_minus_3000_by_2() {
    let mut m = Rp2040::new();
    m.write_u32(SIO_DIV_SDIVIDEND, (-3000i32) as u32);
    assert_eq!(m.read_u32(SIO_DIV_SDIVIDEND) as i32, -3000);
    m.write_u32(SIO_DIV_SDIVISOR, 2);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 3);
    assert_eq!(m.read_u32(SIO_DIV_SDIVISOR) as i32, 2);
    assert_eq!(m.read_u32(SIO_DIV_REMAINDER) as i32, 0);
    assert_eq!(m.read_u32(SIO_DIV_QUOTIENT) as i32, -1500);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 1);
}

#[test]
fn unsigned_divider_123456_by_321() {
    let mut m = Rp2040::new();
    div(&mut m, SIO_DIV_UDIVIDEND, 123456, SIO_DIV_UDIVISOR, 321);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 3);
    assert_eq!((m.read_u32(SIO_DIV_REMAINDER), m.read_u32(SIO_DIV_QUOTIENT)), (192, 384));
    assert_eq!(m.read_u32(SIO_DIV_CSR), 1);
}

#[test]
fn divider_result_save_and_restore() {
    let mut m = Rp2040::new();
    div(&mut m, SIO_DIV_SDIVIDEND, 123456, SIO_DIV_SDIVISOR, -321);
    let remainder = m.read_u32(SIO_DIV_REMAINDER);
    let quotient = m.read_u32(SIO_DIV_QUOTIENT);
    assert_eq!((remainder as i32, quotient as i32), (192, -384));
    div(&mut m, SIO_DIV_UDIVIDEND, 123, SIO_DIV_UDIVISOR, 7);
    assert_eq!((m.read_u32(SIO_DIV_REMAINDER), m.read_u32(SIO_DIV_QUOTIENT)), (4, 17));
    m.write_u32(SIO_DIV_REMAINDER, remainder);
    m.write_u32(SIO_DIV_QUOTIENT, quotient);
    assert_eq!(m.read_u32(SIO_DIV_CSR), 3);
    assert_eq!((m.read_u32(SIO_DIV_REMAINDER) as i32, m.read_u32(SIO_DIV_QUOTIENT) as i32), (192, -384));
}

#[test]
fn divide_by_zero_cases() {
    // (dividend reg, dividend, divisor reg) -> (remainder, quotient). The
    // last upstream case, -3000 / 0 = 1, holds there only because its test
    // driver stores the JavaScript number -3000 in the dividend; through the
    // bus (the only path firmware has) it is 0xFFFFF448 > 0, and the
    // reference returns -1 (see tests.rs, reference_quirk_divide_by_zero).
    for (dreg, a, sreg, rem, q) in [
        (SIO_DIV_UDIVIDEND, 123456i64, SIO_DIV_UDIVISOR, 123456u32, 0xffff_ffffu32),
        (SIO_DIV_UDIVIDEND, 0x8000_0000, SIO_DIV_UDIVISOR, 0x8000_0000, 0xffff_ffff),
        (SIO_DIV_SDIVIDEND, 3000, SIO_DIV_SDIVISOR, 3000, 0xffff_ffff),
        (SIO_DIV_SDIVIDEND, -3000, SIO_DIV_SDIVISOR, (-3000i32) as u32, 0xffff_ffff),
    ] {
        let mut m = Rp2040::new();
        div(&mut m, dreg, a, sreg, 0);
        assert_eq!((m.read_u32(SIO_DIV_REMAINDER), m.read_u32(SIO_DIV_QUOTIENT)), (rem, q), "{a} / 0");
    }
}

#[test]
fn division_of_0x80000000_by_2() {
    let mut m = Rp2040::new();
    div(&mut m, SIO_DIV_SDIVIDEND, 0x8000_0000, SIO_DIV_SDIVISOR, 2);
    assert_eq!((m.read_u32(SIO_DIV_REMAINDER), m.read_u32(SIO_DIV_QUOTIENT)), (0, 0xc000_0000));
    let mut m = Rp2040::new();
    div(&mut m, SIO_DIV_UDIVIDEND, 0x8000_0000, SIO_DIV_UDIVISOR, 2);
    assert_eq!((m.read_u32(SIO_DIV_REMAINDER), m.read_u32(SIO_DIV_QUOTIENT)), (0, 0x4000_0000));
}

#[test]
fn spinlock10_unlock_lock_status() {
    let mut m = Rp2040::new();
    m.write_u32(SIO_SPINLOCK10, 1);
    assert_eq!(m.read_u32(SIO_SPINLOCK10), 1024);
    assert_eq!(m.read_u32(SIO_SPINLOCKST), 1024);
    assert_eq!(m.read_u32(SIO_SPINLOCK10), 0);
    assert_eq!(m.read_u32(SIO_SPINLOCKST), 1024);
    m.write_u32(SIO_SPINLOCK10, 1);
    assert_eq!(m.read_u32(SIO_SPINLOCKST), 0);
}
