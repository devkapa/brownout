//! Unit tests for R1, one group per instruction class and per block.
//!
//! Expected values are derived in each test from the ARMv6-M Architecture
//! Reference Manual's pseudo-code (AddWithCarry, Shift_C, ConditionPassed,
//! PushStack/ExceptionReturn) and the RP2040 datasheet (SIO, SysTick), not
//! read back from the port. Encodings are built here from the ARMv6-M
//! encoding tables.
//!
//! Where rp2040js 1.3.3 departs from ARMv6-M, the port departs with it (the
//! lock-step harness requires that), and a `reference_quirk_*` test pins the
//! reference's behaviour, naming the architectural one beside it.

use super::bus::{FLASH_START, RAM_START};
use super::*;

const CODE: u32 = RAM_START + 0x100;
const STACK: u32 = RAM_START + 0x4_0000;

// ---- ARMv6-M pseudo-code -------------------------------------------------

/// AddWithCarry(x, y, carry_in) -> (result, carry_out, overflow).
fn add_with_carry(x: u32, y: u32, carry: bool) -> (u32, bool, bool) {
    let unsigned_sum = x as u64 + y as u64 + carry as u64;
    let signed_sum = x as i32 as i64 + y as i32 as i64 + carry as i64;
    let result = unsigned_sum as u32;
    (result, result as u64 != unsigned_sum, result as i32 as i64 != signed_sum)
}

/// Shift_C(value, type, amount, carry_in), one bit at a time as the
/// extended-value definitions read.
fn lsl_c(mut x: u32, n: u32, c_in: bool) -> (u32, bool) {
    let mut c = c_in;
    for _ in 0..n {
        c = x >> 31 != 0;
        x <<= 1;
    }
    (x, c)
}
fn lsr_c(mut x: u32, n: u32, c_in: bool) -> (u32, bool) {
    let mut c = c_in;
    for _ in 0..n {
        c = x & 1 != 0;
        x >>= 1;
    }
    (x, c)
}
fn asr_c(mut x: u32, n: u32, c_in: bool) -> (u32, bool) {
    let mut c = c_in;
    for _ in 0..n {
        c = x & 1 != 0;
        x = ((x as i32) >> 1) as u32;
    }
    (x, c)
}
fn ror_c(x: u32, n: u32, c_in: bool) -> (u32, bool) {
    if n == 0 {
        return (x, c_in);
    }
    let m = n % 32;
    let r = (x >> m) | (x.checked_shl(32 - m).unwrap_or(0));
    (r, r >> 31 != 0)
}

/// ConditionPassed for a 4-bit condition and NZCV.
fn condition_passed(cond: u32, n: bool, z: bool, c: bool, v: bool) -> bool {
    match cond {
        0x0 => z,
        0x1 => !z,
        0x2 => c,
        0x3 => !c,
        0x4 => n,
        0x5 => !n,
        0x6 => v,
        0x7 => !v,
        0x8 => c && !z,
        0x9 => !c || z,
        0xa => n == v,
        0xb => n != v,
        0xc => !z && n == v,
        0xd => z || n != v,
        _ => true,
    }
}

const VALUES: [u32; 12] =
    [0, 1, 2, 0x7f, 0x80, 0x7fff_ffff, 0x8000_0000, 0x8000_0001, 0xffff_fffe, 0xffff_ffff, 0x1234_5678, 0xdead_beef];

// ---- Harness for single instructions -------------------------------------

fn machine(code: &[u16]) -> Rp2040 {
    let mut m = Rp2040::new();
    for (i, h) in code.iter().enumerate() {
        let off = (CODE - RAM_START) as usize + 2 * i;
        m.sram[off..off + 2].copy_from_slice(&h.to_le_bytes());
    }
    m.cpu.r[15] = CODE;
    m.cpu.set_sp(STACK);
    m
}

fn flags(m: &Rp2040) -> (bool, bool, bool, bool) {
    (m.cpu.n, m.cpu.z, m.cpu.c, m.cpu.v)
}

fn set_flags(m: &mut Rp2040, n: bool, z: bool, c: bool, v: bool) {
    m.cpu.n = n;
    m.cpu.z = z;
    m.cpu.c = c;
    m.cpu.v = v;
}

// Encodings (ARMv6-M ARM, A6.7).
fn adds_reg(rd: u32, rn: u32, rm: u32) -> u16 {
    (0x1800 | rm << 6 | rn << 3 | rd) as u16
}
fn subs_reg(rd: u32, rn: u32, rm: u32) -> u16 {
    (0x1a00 | rm << 6 | rn << 3 | rd) as u16
}
fn adds_imm3(rd: u32, rn: u32, imm: u32) -> u16 {
    (0x1c00 | imm << 6 | rn << 3 | rd) as u16
}
fn subs_imm8(rdn: u32, imm: u32) -> u16 {
    (0x3800 | rdn << 8 | imm) as u16
}
fn cmp_imm(rn: u32, imm: u32) -> u16 {
    (0x2800 | rn << 8 | imm) as u16
}
fn data_proc(op: u32, rm: u32, rdn: u32) -> u16 {
    (0x4000 | op << 6 | rm << 3 | rdn) as u16
}
const OP_AND: u32 = 0x0;
const OP_EOR: u32 = 0x1;
const OP_LSL: u32 = 0x2;
const OP_LSR: u32 = 0x3;
const OP_ASR: u32 = 0x4;
const OP_ADC: u32 = 0x5;
const OP_SBC: u32 = 0x6;
const OP_ROR: u32 = 0x7;
const OP_TST: u32 = 0x8;
const OP_RSB: u32 = 0x9;
const OP_CMP: u32 = 0xa;
const OP_CMN: u32 = 0xb;
const OP_ORR: u32 = 0xc;
const OP_MUL: u32 = 0xd;
const OP_BIC: u32 = 0xe;
const OP_MVN: u32 = 0xf;
fn shift_imm(kind: u32, rd: u32, rm: u32, imm5: u32) -> u16 {
    (kind << 11 | imm5 << 6 | rm << 3 | rd) as u16
}
fn ldr_imm(rt: u32, rn: u32, imm5: u32) -> u16 {
    (0x6800 | imm5 << 6 | rn << 3 | rt) as u16
}
fn str_imm(rt: u32, rn: u32, imm5: u32) -> u16 {
    (0x6000 | imm5 << 6 | rn << 3 | rt) as u16
}
fn b_cond(cond: u32, imm8: u32) -> u16 {
    (0xd000 | cond << 8 | (imm8 & 0xff)) as u16
}
fn bl(offset: i32) -> [u16; 2] {
    let imm = offset as u32;
    let s = (imm >> 24) & 1;
    let i1 = (imm >> 23) & 1;
    let i2 = (imm >> 22) & 1;
    let j1 = (!(i1 ^ s)) & 1;
    let j2 = (!(i2 ^ s)) & 1;
    let imm10 = (imm >> 12) & 0x3ff;
    let imm11 = (imm >> 1) & 0x7ff;
    [(0xf000 | s << 10 | imm10) as u16, (0xd000 | j1 << 13 | j2 << 11 | imm11) as u16]
}
fn msr(sysm: u32, rn: u32) -> [u16; 2] {
    [(0xf380 | rn) as u16, (0x8800 | sysm) as u16]
}
fn mrs(rd: u32, sysm: u32) -> [u16; 2] {
    [0xf3ef, (0x8000 | rd << 8 | sysm) as u16]
}

// ---- Arithmetic -------------------------------------------------------------

#[test]
fn adds_subs_cmp_cmn_rsbs_match_add_with_carry() {
    for &x in &VALUES {
        for &y in &VALUES {
            // ADDS Rd, Rn, Rm = AddWithCarry(Rn, Rm, 0)
            let mut m = machine(&[adds_reg(2, 0, 1)]);
            m.cpu.r[0] = x;
            m.cpu.r[1] = y;
            assert_eq!(m.execute_instruction(), 1);
            let (r, c, v) = add_with_carry(x, y, false);
            assert_eq!(m.cpu.r[2], r);
            assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v), "ADDS {x:#x} {y:#x}");

            // SUBS Rd, Rn, Rm = AddWithCarry(Rn, NOT(Rm), 1)
            let mut m = machine(&[subs_reg(2, 0, 1)]);
            m.cpu.r[0] = x;
            m.cpu.r[1] = y;
            m.execute_instruction();
            let (r, c, v) = add_with_carry(x, !y, true);
            assert_eq!(m.cpu.r[2], r);
            assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v), "SUBS {x:#x} {y:#x}");

            // CMP and CMN set the flags of SUBS and ADDS, writing nothing.
            let mut m = machine(&[data_proc(OP_CMP, 1, 0)]);
            m.cpu.r[0] = x;
            m.cpu.r[1] = y;
            m.execute_instruction();
            assert_eq!((m.cpu.r[0], m.cpu.r[1]), (x, y));
            assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v));
            let mut m = machine(&[data_proc(OP_CMN, 1, 0)]);
            m.cpu.r[0] = x;
            m.cpu.r[1] = y;
            m.execute_instruction();
            let (r2, c2, v2) = add_with_carry(x, y, false);
            assert_eq!(flags(&m), (r2 >> 31 != 0, r2 == 0, c2, v2));

            for carry in [false, true] {
                // ADCS Rdn, Rm = AddWithCarry(Rdn, Rm, C)
                let mut m = machine(&[data_proc(OP_ADC, 1, 0)]);
                m.cpu.r[0] = x;
                m.cpu.r[1] = y;
                m.cpu.c = carry;
                m.execute_instruction();
                let (r, c, v) = add_with_carry(x, y, carry);
                assert_eq!(m.cpu.r[0], r);
                // Pinned in reference_quirk_adcs_overflow_with_rdn_7fffffff_and_carry.
                let v = if x == 0x7fff_ffff && carry { m.cpu.v } else { v };
                assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v), "ADCS {x:#x} {y:#x} {carry}");
                // SBCS Rdn, Rm = AddWithCarry(Rdn, NOT(Rm), C)
                let mut m = machine(&[data_proc(OP_SBC, 1, 0)]);
                m.cpu.r[0] = x;
                m.cpu.r[1] = y;
                m.cpu.c = carry;
                m.execute_instruction();
                let (r, c, v) = add_with_carry(x, !y, carry);
                assert_eq!(m.cpu.r[0], r);
                // The one input the reference gets V wrong for is pinned in
                // reference_quirk_sbcs_overflow_with_rm_7fffffff_and_no_carry.
                let v = if y == 0x7fff_ffff && !carry { m.cpu.v } else { v };
                assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v), "SBCS {x:#x} {y:#x} {carry}");
            }
        }
        // RSBS Rd, Rn, #0 = AddWithCarry(NOT(Rn), 0, 1)
        let mut m = machine(&[data_proc(OP_RSB, 0, 3)]);
        m.cpu.r[0] = x;
        m.execute_instruction();
        let (r, c, v) = add_with_carry(!x, 0, true);
        assert_eq!(m.cpu.r[3], r);
        assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v));
    }
}

#[test]
fn reference_quirk_sbcs_overflow_with_rm_7fffffff_and_no_carry() {
    // ARMv6-M: SBCS = AddWithCarry(Rdn, NOT(Rm), C). rp2040js computes
    // Rdn - (Rm + 1 - C) and takes V from the sign of that subtrahend, which
    // is itself out of range when Rm = 0x7FFFFFFF and C = 0. For Rdn = 0 the
    // architecture gives V = 0 (0 + 0x80000000 + 0 does not overflow); the
    // reference gives V = 1.
    let mut m = machine(&[data_proc(OP_SBC, 1, 0)]);
    m.cpu.r[0] = 0;
    m.cpu.r[1] = 0x7fff_ffff;
    m.cpu.c = false;
    m.execute_instruction();
    let (r, _, v_arch) = add_with_carry(0, !0x7fff_ffff, false);
    assert_eq!(m.cpu.r[0], r);
    assert!(!v_arch);
    assert!(m.cpu.v);
}

#[test]
fn reference_quirk_adcs_overflow_with_rdn_7fffffff_and_carry() {
    // ARMv6-M: ADCS = AddWithCarry(Rm, Rdn, C). rp2040js adds Rm to
    // (Rdn + C) and takes the signed sum from that second operand truncated
    // to 32 bits, which wraps when Rdn = 0x7FFFFFFF and C = 1. For Rm = 0 the
    // architecture gives V = 1 (0x7FFFFFFF + 1 overflows); the reference
    // gives V = 0.
    let mut m = machine(&[data_proc(OP_ADC, 1, 0)]);
    m.cpu.r[0] = 0x7fff_ffff;
    m.cpu.r[1] = 0;
    m.cpu.c = true;
    m.execute_instruction();
    let (r, _, v_arch) = add_with_carry(0x7fff_ffff, 0, true);
    assert_eq!(m.cpu.r[0], r);
    assert!(v_arch);
    assert!(!m.cpu.v);
}

#[test]
fn immediate_forms_match_add_with_carry() {
    for &x in &VALUES {
        for imm in [0u32, 1, 7] {
            let mut m = machine(&[adds_imm3(1, 0, imm)]);
            m.cpu.r[0] = x;
            m.execute_instruction();
            let (r, c, v) = add_with_carry(x, imm, false);
            assert_eq!((m.cpu.r[1], flags(&m)), (r, (r >> 31 != 0, r == 0, c, v)));
        }
        for imm in [0u32, 1, 0x80, 0xff] {
            let mut m = machine(&[subs_imm8(4, imm)]);
            m.cpu.r[4] = x;
            m.execute_instruction();
            let (r, c, v) = add_with_carry(x, !imm, true);
            assert_eq!((m.cpu.r[4], flags(&m)), (r, (r >> 31 != 0, r == 0, c, v)));
            let mut m = machine(&[cmp_imm(4, imm)]);
            m.cpu.r[4] = x;
            m.execute_instruction();
            assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v));
        }
    }
}

#[test]
fn high_register_add_mov_cmp() {
    // ADD Rdn, Rm (T2) does not touch flags; ADD SP, Rm keeps SP aligned.
    let mut m = machine(&[0x44c8]); // ADD r8, r9: DN=1, Rm=1001, Rdn=000
    m.cpu.r[8] = 5;
    m.cpu.r[9] = 0xffff_ffff;
    set_flags(&mut m, true, true, true, true);
    m.execute_instruction();
    assert_eq!(m.cpu.r[8], 4);
    assert_eq!(flags(&m), (true, true, true, true));
    // MOV PC, Rm: branch, bit 0 cleared, 2 cycles.
    let mut m = machine(&[0x4687]); // MOV pc, r0
    m.cpu.r[0] = RAM_START + 0x201;
    assert_eq!(m.execute_instruction(), 2);
    assert_eq!(m.cpu.pc(), RAM_START + 0x200);
    // MOV Rd, PC reads the instruction's address + 4.
    let mut m = machine(&[0x4678]); // MOV r0, pc
    m.execute_instruction();
    assert_eq!(m.cpu.r[0], CODE + 4);
    // CMP r8, r1 (T2) = flags of r8 - r1.
    let mut m = machine(&[0x4588]); // CMP r8, r1
    m.cpu.r[8] = 3;
    m.cpu.r[1] = 5;
    m.execute_instruction();
    let (r, c, v) = add_with_carry(3, !5, true);
    assert_eq!(flags(&m), (r >> 31 != 0, r == 0, c, v));
}

#[test]
fn sp_and_pc_relative_address_forms() {
    // ADD Rd, SP, #imm8<<2 ; ADD SP, #imm7<<2 ; SUB SP, #imm7<<2 ; ADR.
    let mut m = machine(&[0xa903, 0xb002, 0xb083, 0xa001]);
    m.execute_instruction();
    assert_eq!(m.cpu.r[1], STACK + 12);
    m.execute_instruction();
    assert_eq!(m.cpu.sp(), STACK + 8);
    m.execute_instruction();
    assert_eq!(m.cpu.sp(), STACK + 8 - 12);
    m.execute_instruction(); // ADR r0, #4 at CODE+6: Align(PC,4) + 4 with PC = CODE+10
    assert_eq!(m.cpu.r[0], ((CODE + 6 + 4) & !3) + 4);
}

// ---- Logical, multiply, moves ---------------------------------------------

#[test]
fn logical_ops_set_n_z_and_keep_c_v() {
    let ops: [(u32, fn(u32, u32) -> u32, bool); 7] = [
        (OP_AND, |a, b| a & b, true),
        (OP_EOR, |a, b| a ^ b, true),
        (OP_ORR, |a, b| a | b, true),
        (OP_BIC, |a, b| a & !b, true),
        (OP_MVN, |_, b| !b, true),
        (OP_MUL, |a, b| a.wrapping_mul(b), true),
        (OP_TST, |a, b| a & b, false),
    ];
    for &(op, f, writes) in &ops {
        for &x in &VALUES {
            for &y in &VALUES {
                for cv in [false, true] {
                    let mut m = machine(&[data_proc(op, 1, 0)]);
                    m.cpu.r[0] = x;
                    m.cpu.r[1] = y;
                    m.cpu.c = cv;
                    m.cpu.v = !cv;
                    assert_eq!(m.execute_instruction(), 1);
                    let r = f(x, y);
                    assert_eq!(m.cpu.r[0], if writes { r } else { x }, "op {op} {x:#x} {y:#x}");
                    assert_eq!(flags(&m), (r >> 31 != 0, r == 0, cv, !cv), "op {op} {x:#x} {y:#x}");
                }
            }
        }
    }
    // MOVS Rd, #imm8: N=0, Z from the value, C and V kept.
    let mut m = machine(&[0x2000, 0x21ff]);
    set_flags(&mut m, true, false, true, true);
    m.execute_instruction();
    assert_eq!((m.cpu.r[0], flags(&m)), (0, (false, true, true, true)));
    m.execute_instruction();
    assert_eq!((m.cpu.r[1], flags(&m)), (0xff, (false, false, true, true)));
}

#[test]
fn extend_and_reverse() {
    for &x in &VALUES {
        // Rd = r1, Rm = r0 in every encoding.
        let cases: [(u16, u32); 7] = [
            (0xb201, x as u16 as i16 as i32 as u32),                         // SXTH
            (0xb241, x as u8 as i8 as i32 as u32),                           // SXTB
            (0xb281, x & 0xffff),                                            // UXTH
            (0xb2c1, x & 0xff),                                              // UXTB
            (0xba01, (x << 24) | ((x << 8) & 0xff_0000) | ((x >> 8) & 0xff00) | (x >> 24)), // REV
            (0xba41, ((x & 0x00ff_00ff) << 8) | ((x >> 8) & 0x00ff_00ff)), // REV16
            (0xbac1, (((x & 0xff) << 8 | (x >> 8) & 0xff) as u16 as i16 as i32) as u32), // REVSH
        ];
        for &(op, want) in cases.iter() {
            let mut m = machine(&[op]);
            m.cpu.r[0] = x;
            m.execute_instruction();
            assert_eq!(m.cpu.r[1], want, "op {op:#x} x {x:#x}");
        }
    }
}

// ---- Shifts -------------------------------------------------------------------

#[test]
fn shifts_by_immediate_match_shift_c() {
    for &x in &VALUES {
        for imm5 in 0..32u32 {
            for c_in in [false, true] {
                // LSL #0 is MOVS (C kept); LSR/ASR #0 encode a shift of 32.
                let cases = [
                    (0u32, lsl_c(x, imm5, c_in)),
                    (1, lsr_c(x, if imm5 == 0 { 32 } else { imm5 }, c_in)),
                    (2, asr_c(x, if imm5 == 0 { 32 } else { imm5 }, c_in)),
                ];
                for (kind, (r, c)) in cases {
                    let mut m = machine(&[shift_imm(kind, 1, 0, imm5)]);
                    m.cpu.r[0] = x;
                    m.cpu.c = c_in;
                    m.execute_instruction();
                    assert_eq!(m.cpu.r[1], r, "kind {kind} x {x:#x} imm {imm5}");
                    assert_eq!((m.cpu.n, m.cpu.z, m.cpu.c), (r >> 31 != 0, r == 0, c), "kind {kind} x {x:#x} imm {imm5}");
                }
            }
        }
    }
}

#[test]
fn shifts_by_register_match_shift_c_where_the_reference_is_architectural() {
    for &x in &VALUES {
        for amount in [1u32, 2, 15, 31, 32, 33, 40, 255, 0x1_00 | 4] {
            for c_in in [false, true] {
                let n = amount & 0xff;
                let cases = [
                    (OP_LSL, lsl_c(x, n, c_in), n <= 32),
                    (OP_LSR, lsr_c(x, n, c_in), true),
                    (OP_ASR, asr_c(x, n, c_in), true),
                    (OP_ROR, ror_c(x, n, c_in), true),
                ];
                for (op, (r, c), architectural) in cases {
                    let mut m = machine(&[data_proc(op, 1, 0)]);
                    m.cpu.r[0] = x;
                    m.cpu.r[1] = amount;
                    m.cpu.c = c_in;
                    m.execute_instruction();
                    assert_eq!(m.cpu.r[0], r, "op {op} x {x:#x} n {n}");
                    assert_eq!((m.cpu.n, m.cpu.z), (r >> 31 != 0, r == 0));
                    if architectural {
                        assert_eq!(m.cpu.c, c, "op {op} x {x:#x} n {n}");
                    }
                }
            }
        }
    }
}

#[test]
fn reference_quirk_register_shift_by_zero_sets_c_from_bit_31() {
    // ARMv6-M: Shift_C with amount 0 returns carry_in. rp2040js tests bit
    // (0 - 1) & 31 = 31 for LSRS and ASRS, and bit 31 of the result for RORS.
    for op in [OP_LSR, OP_ASR, OP_ROR] {
        for &x in &VALUES {
            let mut m = machine(&[data_proc(op, 1, 0)]);
            m.cpu.r[0] = x;
            m.cpu.r[1] = 0x100; // Rm[7:0] = 0
            m.cpu.c = x >> 31 == 0; // the opposite of what the quirk sets
            m.execute_instruction();
            assert_eq!(m.cpu.r[0], x);
            assert_eq!(m.cpu.c, x >> 31 != 0, "op {op} x {x:#x}");
        }
    }
    // LSLS by register with 0 keeps C, as the architecture says.
    let mut m = machine(&[data_proc(OP_LSL, 1, 0)]);
    m.cpu.r[0] = 0x8000_0000;
    m.cpu.c = false;
    m.execute_instruction();
    assert!(!m.cpu.c);
}

#[test]
fn reference_quirk_lsls_register_past_32_tests_a_wrapped_bit() {
    // ARMv6-M: LSL_C by 33..255 gives C = 0. rp2040js tests bit (32 - n) & 31.
    for n in [33u32, 40, 63, 255] {
        let x = 0xffff_ffffu32;
        let mut m = machine(&[data_proc(OP_LSL, 1, 0)]);
        m.cpu.r[0] = x;
        m.cpu.r[1] = n;
        m.execute_instruction();
        assert_eq!(m.cpu.r[0], 0);
        assert_eq!(m.cpu.c, (x >> (32u32.wrapping_sub(n) & 31)) & 1 != 0);
        assert!(m.cpu.c, "the architecture would give C = 0 here");
    }
}

// ---- Branches -----------------------------------------------------------------

#[test]
fn conditional_branches_follow_condition_passed() {
    for cond in 0..14u32 {
        for nzcv in 0..16u32 {
            let (n, z, c, v) = (nzcv & 8 != 0, nzcv & 4 != 0, nzcv & 2 != 0, nzcv & 1 != 0);
            let mut m = machine(&[b_cond(cond, 0xfe)]); // offset -4 from PC+4
            set_flags(&mut m, n, z, c, v);
            let cycles = m.execute_instruction();
            let taken = condition_passed(cond, n, z, c, v);
            if taken {
                assert_eq!((m.cpu.pc(), cycles), (CODE + 4 - 4, 2), "cond {cond} nzcv {nzcv:04b}");
            } else {
                assert_eq!((m.cpu.pc(), cycles), (CODE + 2, 1), "cond {cond} nzcv {nzcv:04b}");
            }
        }
    }
}

#[test]
fn unconditional_branch_bl_bx_blx() {
    // B: PC + 4 + SignExtend(imm11:0).
    let mut m = machine(&[0xe7fe]); // B . (offset -4)
    assert_eq!(m.execute_instruction(), 2);
    assert_eq!(m.cpu.pc(), CODE);
    let mut m = machine(&[0xe002]); // B +4
    m.execute_instruction();
    assert_eq!(m.cpu.pc(), CODE + 4 + 4);
    // BL: LR = next instruction | 1, PC = PC + 4 + offset, 3 cycles.
    for off in [0x10i32, -0x100, 0x3f_fffe, -0x40_0000] {
        let mut m = machine(&bl(off));
        assert_eq!(m.execute_instruction(), 3);
        assert_eq!(m.cpu.r[14], (CODE + 4) | 1);
        assert_eq!(m.cpu.pc(), (CODE + 4).wrapping_add(off as u32), "BL {off:#x}");
    }
    // BX: PC = Rm with bit 0 cleared, 2 cycles; BLX also sets LR.
    let mut m = machine(&[0x4700 | (2 << 3)]); // BX r2
    m.cpu.r[2] = RAM_START + 0x301;
    assert_eq!(m.execute_instruction(), 2);
    assert_eq!(m.cpu.pc(), RAM_START + 0x300);
    let mut m = machine(&[0x4780 | (2 << 3)]); // BLX r2
    m.cpu.r[2] = RAM_START + 0x301;
    assert_eq!(m.execute_instruction(), 2);
    assert_eq!((m.cpu.pc(), m.cpu.r[14]), (RAM_START + 0x300, (CODE + 2) | 1));
}

// ---- Loads and stores -----------------------------------------------------------

#[test]
fn word_halfword_byte_loads_and_stores_are_little_endian() {
    let base = RAM_START + 0x1000;
    let mut m = machine(&[
        str_imm(0, 1, 1),                 // STR r0, [r1, #4]
        0x8000 | (1 << 6) | (1 << 3) | 2, // STRH r2, [r1, #2]
        0x7000 | (1 << 3) | 3,            // STRB r3, [r1, #0]
        ldr_imm(4, 1, 1),                 // LDR r4, [r1, #4]
        0x8800 | (1 << 6) | (1 << 3) | 5, // LDRH r5, [r1, #2]
        0x7800 | (1 << 3) | 6,            // LDRB r6, [r1, #0]
    ]);
    m.cpu.r[0] = 0x1122_3344;
    m.cpu.r[1] = base;
    m.cpu.r[2] = 0xaaaa_5566;
    m.cpu.r[3] = 0xffff_ff77;
    for _ in 0..3 {
        assert_eq!(m.execute_instruction(), 2); // stores to SRAM: 2 cycles
    }
    let o = (base - RAM_START) as usize;
    assert_eq!(&m.sram[o..o + 8], &[0x77, 0x00, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
    for _ in 0..3 {
        assert_eq!(m.execute_instruction(), 2);
    }
    assert_eq!((m.cpu.r[4], m.cpu.r[5], m.cpu.r[6]), (0x1122_3344, 0x5566, 0x77));
}

#[test]
fn signed_loads_register_offset_and_literal() {
    let base = RAM_START + 0x2000;
    let mut m = machine(&[
        0x5600 | (2 << 6) | (1 << 3) | 3, // LDRSB r3, [r1, r2]
        0x5e00 | (2 << 6) | (1 << 3) | 4, // LDRSH r4, [r1, r2]
        0x5800 | (2 << 6) | (1 << 3) | 5, // LDR r5, [r1, r2]
        0x4801,                           // LDR r0, [PC, #4]
        0x0000,
        0x0000,
        0xbeef,
        0xdead,
    ]);
    let o = (base + 4 - RAM_START) as usize;
    m.sram[o..o + 4].copy_from_slice(&0x8081_f2f3u32.to_le_bytes());
    m.cpu.r[1] = base;
    m.cpu.r[2] = 4;
    m.execute_instruction();
    m.execute_instruction();
    m.execute_instruction();
    assert_eq!(m.cpu.r[3], 0xf3u8 as i8 as i32 as u32);
    assert_eq!(m.cpu.r[4], 0xf2f3u16 as i16 as i32 as u32);
    assert_eq!(m.cpu.r[5], 0x8081_f2f3);
    // LDR literal at CODE+6: Align(PC = CODE+10, 4) + 4 = CODE+12.
    m.execute_instruction();
    assert_eq!(m.cpu.r[0], 0xdead_beef);
}

#[test]
fn ldm_stm_push_pop() {
    let base = RAM_START + 0x3000;
    // STMIA r0!, {r1, r2, r3}; LDMIA r4!, {r5, r6, r7}
    let mut m = machine(&[0xc00e, 0xcce0]);
    m.cpu.r[0] = base;
    m.cpu.r[1] = 1;
    m.cpu.r[2] = 2;
    m.cpu.r[3] = 3;
    m.cpu.r[4] = base;
    assert_eq!(m.execute_instruction(), 1 + 3);
    assert_eq!(m.cpu.r[0], base + 12);
    assert_eq!(m.execute_instruction(), 1 + 3);
    assert_eq!((m.cpu.r[5], m.cpu.r[6], m.cpu.r[7], m.cpu.r[4]), (1, 2, 3, base + 12));
    // LDMIA r0, {r0, r1}: Rn in the list, no write-back; r0 gets the load.
    let mut m = machine(&[0xc803]);
    let o = (base - RAM_START) as usize;
    m.sram[o..o + 8].copy_from_slice(&[9, 0, 0, 0, 8, 0, 0, 0]);
    m.cpu.r[0] = base;
    m.execute_instruction();
    assert_eq!((m.cpu.r[0], m.cpu.r[1]), (9, 8));

    // PUSH {r0, r1, lr}; POP {r2, r3, pc}
    let mut m = machine(&[0xb503, 0xbd0c]);
    m.cpu.r[0] = 0xa0;
    m.cpu.r[1] = 0xa1;
    m.cpu.r[14] = RAM_START + 0x401;
    m.execute_instruction();
    assert_eq!(m.cpu.sp(), STACK - 12);
    let o = (STACK - 12 - RAM_START) as usize;
    assert_eq!(&m.sram[o..o + 12], &[0xa0, 0, 0, 0, 0xa1, 0, 0, 0, 0x01, 0x04, 0, 0x20]);
    // POP with PC: 1 + N + 2 cycles in the reference (N low registers).
    assert_eq!(m.execute_instruction(), 1 + 2 + 2);
    assert_eq!((m.cpu.r[2], m.cpu.r[3], m.cpu.pc(), m.cpu.sp()), (0xa0, 0xa1, RAM_START + 0x400, STACK));
}

#[test]
fn reference_quirk_push_counts_low_registers_only_and_stm_skips_writeback() {
    // Cortex-M0+ TRM: PUSH takes 1 + N cycles counting LR. rp2040js counts
    // only r0..r7.
    let mut m = machine(&[0xb501]); // PUSH {r0, lr}
    assert_eq!(m.execute_instruction(), 2);
    // ARMv6-M STM always writes back (Rn in the list but not lowest is
    // UNKNOWN). rp2040js never writes back when Rn is in the list.
    let mut m = machine(&[0xc103]); // STMIA r1!, {r0, r1}
    let base = RAM_START + 0x3100;
    m.cpu.r[1] = base;
    m.execute_instruction();
    assert_eq!(m.cpu.r[1], base);
}

#[test]
fn load_cycles_by_region() {
    // SIO is single-cycle I/O: LDR from it takes 1 cycle; SRAM 2.
    let mut m = machine(&[ldr_imm(0, 1, 0), ldr_imm(0, 2, 0)]);
    m.cpu.r[1] = 0xd000_0000; // SIO CPUID
    m.cpu.r[2] = RAM_START;
    assert_eq!(m.execute_instruction(), 1);
    assert_eq!(m.execute_instruction(), 2);
}

// ---- Exceptions and the NVIC ------------------------------------------------------

/// A machine whose vector table sits at RAM_START with every handler at
/// HANDLER (a `BX LR`).
const HANDLER: u32 = RAM_START + 0x800;
fn machine_with_vectors(code: &[u16]) -> Rp2040 {
    let mut m = machine(code);
    m.cpu.vtor = RAM_START;
    for n in 1..48u32 {
        let o = (4 * n) as usize;
        m.sram[o..o + 4].copy_from_slice(&(HANDLER | 1).to_le_bytes());
    }
    let h = (HANDLER - RAM_START) as usize;
    m.sram[h..h + 2].copy_from_slice(&0x4770u16.to_le_bytes()); // BX LR
    m
}

/// The ARMv6-M PushStack frame for a thread-mode exception on the main
/// stack: (frame address, eight words).
fn expected_frame(sp: u32, r: [u32; 4], r12: u32, lr: u32, ret: u32, xpsr: u32) -> (u32, [u32; 8]) {
    let align = (sp >> 2) & 1;
    let frame = (sp - 0x20) & !4;
    (frame, [r[0], r[1], r[2], r[3], r12, lr, ret, (xpsr & !(1 << 9)) | (align << 9)])
}

fn words_at(m: &Rp2040, addr: u32) -> [u32; 8] {
    let mut w = [0u32; 8];
    for (i, v) in w.iter_mut().enumerate() {
        let o = (addr - RAM_START) as usize + 4 * i;
        *v = u32::from_le_bytes([m.sram[o], m.sram[o + 1], m.sram[o + 2], m.sram[o + 3]]);
    }
    w
}

#[test]
fn svc_stacks_the_frame_and_returns() {
    for sp in [STACK, STACK - 4] {
        let mut m = machine_with_vectors(&[0xdf05, 0xbf00]); // SVC #5; NOP
        m.cpu.set_sp(sp);
        m.cpu.r[..4].copy_from_slice(&[10, 11, 12, 13]);
        m.cpu.r[12] = 14;
        m.cpu.r[14] = 0x1234_5679;
        set_flags(&mut m, true, false, true, false);
        let xpsr = m.cpu.xpsr();
        m.execute_instruction(); // SVC pends SVCall
        m.execute_instruction(); // taken before the next instruction; BX LR runs
        let (frame, words) = expected_frame(sp, [10, 11, 12, 13], 14, 0x1234_5679, CODE + 2, xpsr);
        assert_eq!(words_at(&m, frame), words);
        assert!(!m.cpu.handler_mode, "BX LR with EXC_RETURN 0xFFFFFFF9 returns to thread");
        assert_eq!(m.cpu.ipsr, 0);
        assert_eq!(m.cpu.pc(), CODE + 2);
        assert_eq!(m.cpu.sp(), sp, "the frame and its alignment padding are popped");
        assert_eq!(m.cpu.r[14], 0x1234_5679);
        assert_eq!(flags(&m), (true, false, true, false));
    }
}

#[test]
fn exception_entry_sets_ipsr_lr_and_vector() {
    let mut m = machine_with_vectors(&[0xdf00]);
    m.execute_instruction();
    // Run only the entry: check before the handler's BX executes by
    // taking the exception directly.
    assert!(m.check_for_interrupts());
    assert_eq!((m.cpu.ipsr, m.cpu.r[14], m.cpu.pc(), m.cpu.handler_mode), (11, 0xffff_fff9, HANDLER | 1, true));
}

#[test]
fn nvic_priorities_order_pending_interrupts() {
    let mut m = machine_with_vectors(&[0xbf00; 8]);
    // Enable IRQs 26 and 27 (software-only lines), give 27 the higher
    // priority (lower number), pend both.
    m.ppb_write(super::ppb::NVIC_ISER, (1 << 26) | (1 << 27));
    m.ppb_write(super::ppb::NVIC_IPR0 + 24, (3 << 22) | (1 << 30)); // IPR6: IRQ26 byte 2 = 3, IRQ27 byte 3 = 1
    assert_eq!(m.cpu.exception_priority(16 + 26), 3);
    assert_eq!(m.cpu.exception_priority(16 + 27), 1);
    m.ppb_write(super::ppb::NVIC_ISPR, (1 << 26) | (1 << 27));
    // ICSR.VECTPENDING names the higher-priority one.
    let icsr = m.ppb_read(super::ppb::ICSR);
    assert_eq!((icsr >> 12) & 0x1ff, 16 + 27);
    assert!(icsr & (1 << 22) != 0);
    assert!(m.check_for_interrupts());
    assert_eq!(m.cpu.ipsr, 16 + 27);
    // Entry clears a software line's pending bit (IRQ > 25).
    assert_eq!(m.cpu.pending_interrupts, 1 << 26);
    // While 27 (priority 1) is active, 26 (priority 3) cannot preempt.
    m.cpu.interrupts_updated = true;
    assert!(!m.check_for_interrupts());
}

#[test]
fn primask_masks_configurable_interrupts_and_cpsie_reenables() {
    let mut m = machine_with_vectors(&[0xb672, 0xbf00, 0xb662, 0xbf00]); // CPSID i; NOP; CPSIE i; NOP
    m.ppb_write(super::ppb::NVIC_ISER, 1 << 30);
    m.execute_instruction();
    m.ppb_write(super::ppb::NVIC_ISPR, 1 << 30);
    m.execute_instruction(); // masked: the NOP runs
    assert_eq!(m.cpu.ipsr, 0);
    assert_eq!(m.cpu.pc(), CODE + 4);
    m.execute_instruction(); // CPSIE
    m.execute_instruction(); // taken now; the handler's BX LR runs
    assert!(!m.cpu.handler_mode);
    assert_eq!(m.cpu.pc(), CODE + 6);
}

#[test]
fn process_stack_thread_returns_with_exc_return_fffffffd() {
    let psp = RAM_START + 0x2_0000;
    let mut code = vec![];
    code.extend_from_slice(&msr(9, 0)); // MSR PSP, r0
    code.push(0x2102); // MOVS r1, #2
    code.extend_from_slice(&msr(20, 1)); // MSR CONTROL, r1 (SPSEL = 1)
    code.push(0xdf00); // SVC 0
    code.push(0xbf00);
    let mut m = machine_with_vectors(&code);
    m.cpu.r[0] = psp;
    assert_eq!(m.execute_instruction(), 3);
    m.execute_instruction();
    assert_eq!(m.execute_instruction(), 3);
    assert_eq!(m.cpu.sp(), psp);
    assert_eq!(m.cpu.sp_main(), STACK);
    m.execute_instruction(); // SVC
    // Entry: frame on the process stack, handler on main, LR = 0xFFFFFFFD.
    assert!(m.check_for_interrupts());
    assert_eq!(m.cpu.r[14], 0xffff_fffd);
    assert_eq!(m.cpu.sp(), STACK);
    assert_eq!(m.cpu.sp_process(), psp - 0x20);
    m.execute_instruction(); // BX LR
    assert_eq!((m.cpu.sp(), m.cpu.spsel, m.cpu.handler_mode), (psp, true, false));
    // MRS reads back CONTROL and PSP.
    let mut m2 = machine(&[mrs(3, 20)[0], mrs(3, 20)[1]]);
    m2.cpu.spsel = true;
    m2.execute_instruction();
    assert_eq!(m2.cpu.r[3], 2);
}

#[test]
fn pendsv_and_systick_pend_through_icsr() {
    let mut m = machine_with_vectors(&[0xbf00; 4]);
    m.ppb_write(super::ppb::ICSR, 1 << 28);
    assert!(m.cpu.pending_pendsv);
    assert!(m.check_for_interrupts());
    assert_eq!(m.cpu.ipsr, 14);
    let mut m = machine_with_vectors(&[0xbf00; 4]);
    m.ppb_write(super::ppb::ICSR, 1 << 26);
    m.ppb_write(super::ppb::ICSR, 1 << 25);
    assert!(!m.cpu.pending_systick, "PENDSTCLR clears it");
}

#[test]
fn wfi_waits_and_an_interrupt_line_wakes_it() {
    let mut m = machine_with_vectors(&[0xbf30, 0xbf00]); // WFI; NOP
    m.ppb_write(super::ppb::NVIC_ISER, 1 << 5);
    assert_eq!(m.execute_instruction(), 2);
    assert!(m.cpu.waiting);
    m.set_interrupt(5, true);
    assert!(!m.cpu.waiting);
    assert_eq!(m.cpu.ipsr, 16 + 5);
}

#[test]
fn wfe_consumes_a_registered_event() {
    let mut m = machine(&[0xbf20, 0xbf20]);
    m.cpu.event_registered = true;
    m.execute_instruction();
    assert!(!m.cpu.waiting && !m.cpu.event_registered);
    m.execute_instruction();
    assert!(m.cpu.waiting);
}

#[test]
fn bkpt_and_udf_stop_with_their_code() {
    let mut m = machine(&[0xbe2a]);
    m.step();
    assert_eq!(m.stop, Some(Stop::Break { code: 0x2a }));
    assert_eq!(m.steps, 1, "the breaking instruction completed");
    let mut m = machine(&[0xde07]);
    m.step();
    assert_eq!(m.stop, Some(Stop::Break { code: 7 }));
}

// ---- Bus and memory map ------------------------------------------------------------

#[test]
fn memory_map_regions() {
    let mut m = Rp2040::new();
    m.bootrom[1] = 0xcafe_f00d;
    assert_eq!(m.read_u32(4), 0xcafe_f00d);
    m.flash[0x10..0x14].copy_from_slice(&0x0102_0304u32.to_le_bytes());
    // Flash is read through four mirrors.
    for mirror in 0..4 {
        assert_eq!(m.read_u32(FLASH_START + mirror * 0x0100_0000 + 0x10), 0x0102_0304);
    }
    assert_eq!(m.read_u16(FLASH_START + 0x12), 0x0102);
    assert_eq!(m.read_u8(FLASH_START + 0x13), 0x01);
    m.write_u32(RAM_START + 0x4_1ffc, 0xa1b2_c3d4); // last SRAM word (264 KB)
    assert_eq!(m.read_u32(RAM_START + 0x4_1ffc), 0xa1b2_c3d4);
    assert_eq!(m.read_u32(RAM_START + 0x4_2000), 0xffff_ffff, "past SRAM is unmapped");
    assert!(m.stop.is_none());
    // SIO CPUID is core 0; PPB CPUID is a Cortex-M0+ r0p1.
    assert_eq!(m.read_u32(0xd000_0000), 0);
    assert_eq!(m.read_u32(0xe000_ed00), 0x410c_c601);
}

#[test]
fn unported_peripherals_stop_the_machine_at_first_access() {
    for (addr, name, write) in [
        (0x4004_0000u32, "SPI1", false),
        (0x4004_4000, "I2C0", true),
    ] {
        let mut m = Rp2040::new();
        if write {
            m.write_u32(addr, 1);
        } else {
            m.read_u32(addr);
        }
        assert_eq!(m.stop, Some(Stop::Peripheral { name, address: addr, write }));
    }
    // R4: the USB DPRAM stores the word and runs the controller's hook.
    let mut m = Rp2040::new();
    m.write_u32(0x5010_0000, 1);
    assert_eq!(m.stop, None);
    assert_eq!(m.read_u32(0x5010_0000), 1);
    let mut m = Rp2040::new();
    m.read_u32(RAM_START + 2);
    assert_eq!(m.stop, Some(Stop::UnalignedRead { address: RAM_START + 2 }));
}

#[test]
fn byte_store_outside_sram_is_a_word_read_modify_write() {
    // A byte store to a spinlock register reads it first (claiming the
    // lock), then writes the word back (releasing it).
    let mut m = Rp2040::new();
    m.write_u8(0xd000_0101, 0xff);
    assert_eq!(m.sio.spin_lock, 0);
    // A byte store to flash merges into the word.
    m.flash[0..4].copy_from_slice(&[1, 2, 3, 4]);
    m.write_u8(FLASH_START + 2, 0xee);
    assert_eq!(&m.flash[0..4], &[1, 2, 0xee, 4]);
    m.write_u16(FLASH_START + 0, 0xbbaa);
    assert_eq!(&m.flash[0..4], &[0xaa, 0xbb, 0xee, 4]);
}

// ---- SIO --------------------------------------------------------------------------

#[test]
fn divider_matches_integer_division_and_costs_8_cycles() {
    let pairs = [(100u32, 7u32), (7, 100), (0xffff_ffff, 3), (0x8000_0000, 0xffff_ffff), (12345, 1)];
    for (a, b) in pairs {
        let mut m = Rp2040::new();
        m.sio_write(0x060, a); // UDIVIDEND
        m.sio_write(0x064, b); // UDIVISOR
        assert_eq!(m.cpu.cycles, 16);
        assert_eq!(m.sio_read(0x078), 0b11, "CSR: ready and dirty");
        assert_eq!(m.sio_read(0x074), a % b);
        assert_eq!(m.sio_read(0x070), a / b);
        assert_eq!(m.sio_read(0x078), 0b01, "reading the quotient clears dirty");
        let mut m = Rp2040::new();
        m.sio_write(0x068, a); // SDIVIDEND
        m.sio_write(0x06c, b); // SDIVISOR
        let (sa, sb) = (a as i32 as i64, b as i32 as i64);
        assert_eq!(m.sio_read(0x070), (sa / sb) as u32, "{sa} / {sb}");
        assert_eq!(m.sio_read(0x074), (sa % sb) as u32);
    }
}

#[test]
fn reference_quirk_divide_by_zero() {
    // RP2040 datasheet 2.3.1.5: quotient -1 for a non-negative signed
    // dividend, 1 for a negative one. rp2040js tests the stored dividend as
    // unsigned: -1 for any non-zero dividend, 1 for zero, in both modes.
    for (dividend, q) in [(5u32, 0xffff_ffffu32), (0, 1), (0x8000_0000, 0xffff_ffff)] {
        let mut m = Rp2040::new();
        m.sio_write(0x068, dividend);
        m.sio_write(0x06c, 0);
        assert_eq!((m.sio_read(0x070), m.sio_read(0x074)), (q, dividend));
    }
}

#[test]
fn spinlocks_claim_on_read_and_release_on_write() {
    let mut m = Rp2040::new();
    for n in [0u32, 5, 31] {
        let reg = 0x100 + 4 * n;
        assert_eq!(m.sio_read(reg), 1 << n, "free lock: the read claims it");
        assert_eq!(m.sio_read(reg), 0, "held lock reads 0");
        assert!(m.sio_read(0x05c) & (1 << n) != 0);
        m.sio_write(reg, 0);
        assert_eq!(m.sio_read(0x05c) & (1 << n), 0);
    }
}

#[test]
fn gpio_out_and_oe_set_clr_xor() {
    let mut m = Rp2040::new();
    m.sio_write(0x010, 0xffff_ffff);
    assert_eq!(m.sio_read(0x010), 0x3fff_ffff, "30 GPIOs");
    m.sio_write(0x018, 0x0000_00f0); // CLR
    m.sio_write(0x01c, 0x0000_0003); // XOR
    assert_eq!(m.sio_read(0x010), 0x3fff_ff0c);
    m.sio_write(0x024, 1 << 25); // OE SET
    assert_eq!(m.sio_read(0x020), 1 << 25);
    m.sio_write(0x02c, 1 << 25); // OE XOR
    assert_eq!(m.sio_read(0x020), 0);
    // The SET/CLR/XOR aliases read 0; GPIO_IN reads 0 with input buffers
    // disabled at reset (pads IE = 0).
    assert_eq!(m.sio_read(0x014), 0);
    m.set_input_value(3, true);
    assert_eq!(m.sio_read(0x004), 0);
}

#[test]
fn reference_quirk_first_pin_update_reports_input_then_pulldown() {
    // The first SIO write that touches a pin reports a change from Input to
    // its real reset state (pull-down), because the reference remembers the
    // state it computed before the pin's pad register existed.
    let mut m = Rp2040::new();
    m.sio_write(0x014, 1 << 7);
    assert_eq!(m.pin_events, vec![PinEvent { pin: 7, state: sio::PIN_INPUT_PULLDOWN, nanos: 0.0 }]);
    m.sio_write(0x018, 1 << 7);
    assert_eq!(m.pin_events.len(), 1, "an unselected function leaves the pin's state unchanged");
    // GPIO1 is the exception: the reference's PWM reset already refreshed it.
    m.sio_write(0x014, 1 << 1);
    assert_eq!(m.pin_events.len(), 1);
}

#[test]
fn interpolator_lanes_shift_mask_add_and_pop() {
    // Datasheet 2.3.1.6: lane result = ((accum >> shift) & mask), sign
    // extended from MASK_MSB if SIGNED, plus BASE; POP writes the lane
    // results back to the accumulators.
    let mut m = Rp2040::new();
    let ctrl = |shift: u32, lsb: u32, msb: u32, signed: bool| shift | lsb << 5 | msb << 10 | (signed as u32) << 15;
    m.sio_write(0x0ac, ctrl(4, 0, 7, false)); // INTERP0 CTRL_LANE0
    m.sio_write(0x0b0, ctrl(0, 0, 15, true)); // CTRL_LANE1
    m.sio_write(0x080, 0x0000_1234); // ACCUM0
    m.sio_write(0x084, 0x0000_8001); // ACCUM1
    m.sio_write(0x088, 100); // BASE0
    m.sio_write(0x08c, 0); // BASE1
    m.sio_write(0x090, 7); // BASE2
    let lane0 = (0x1234u32 >> 4) & 0xff;
    let lane1 = 0x8001u16 as i16 as i32 as u32;
    assert_eq!(m.sio_read(0x0a0), lane0 + 100); // PEEK_LANE0
    assert_eq!(m.sio_read(0x0a4), lane1); // PEEK_LANE1
    assert_eq!(m.sio_read(0x0a8), 7u32.wrapping_add(lane0).wrapping_add(lane1)); // PEEK_FULL
    assert_eq!(m.sio_read(0x0b4), lane0, "ACCUM0_ADD reads the lane result before BASE");
    // POP_LANE0 returns lane 0 and writes both lanes back.
    assert_eq!(m.sio_read(0x094), lane0 + 100);
    assert_eq!(m.sio_read(0x080), lane0 + 100);
    assert_eq!(m.sio_read(0x084), lane1);
    // ACCUM0_ADD accumulates.
    m.sio_write(0x0b4, 5);
    assert_eq!(m.sio_read(0x080), lane0 + 105);
}

#[test]
fn interpolator1_clamps_and_interpolator0_blends() {
    // INTERP1 CLAMP: lane 0 result clamped to [BASE0, BASE1].
    let mut m = Rp2040::new();
    m.sio_write(0x0ec, 31 << 10 | 1 << 22); // INTERP1 CTRL_LANE0: MASK_MSB 31, CLAMP
    m.sio_write(0x0c8, 10); // BASE0
    m.sio_write(0x0cc, 20); // BASE1
    for (acc, want) in [(5u32, 10u32), (15, 15), (25, 20)] {
        m.sio_write(0x0c0, acc);
        assert_eq!(m.sio_read(0x0e0), want, "clamp {acc}");
    }
    // INTERP0 BLEND: lane 1 = BASE0 + alpha * (BASE1 - BASE0) / 256 with
    // alpha the low 8 bits of lane 1's shift-and-mask result.
    let mut m = Rp2040::new();
    m.sio_write(0x0ac, 31 << 10 | 1 << 21); // CTRL_LANE0: BLEND
    m.sio_write(0x0b0, 31 << 10); // CTRL_LANE1
    m.sio_write(0x088, 1000); // BASE0
    m.sio_write(0x08c, 2000); // BASE1
    m.sio_write(0x084, 64); // ACCUM1: alpha 64/256
    assert_eq!(m.sio_read(0x0a4), 1000 + 64 * 1000 / 256);
    assert_eq!(m.sio_read(0x0a0), 64, "lane 0 shows alpha in blend mode");
}

// ---- SysTick --------------------------------------------------------------------------

#[test]
fn systick_counts_down_at_125_mhz_and_sets_countflag() {
    let mut m = Rp2040::new();
    m.ppb_write(super::ppb::SYST_RVR, 999);
    m.ppb_write(super::ppb::SYST_CVR, 0);
    m.ppb_write(super::ppb::SYST_CSR, 0b111); // enable, interrupt, processor clock
    // From 0, the first tick reloads; the counter falls one per 8 ns cycle.
    m.clock_tick(8.0 * 10.0);
    let cvr = m.ppb_read(super::ppb::SYST_CVR);
    assert!(cvr <= 999 && cvr >= 989, "cvr {cvr}");
    assert!(m.ppb_read(super::ppb::SYST_CSR) & (1 << 16) != 0, "COUNTFLAG after reaching 0");
    assert_eq!(m.ppb_read(super::ppb::SYST_CSR) & (1 << 16), 0, "COUNTFLAG clears on read");
    assert!(m.cpu.pending_systick);
    m.cpu.pending_systick = false;
    // One full period (1000 cycles) later it pends again.
    m.clock_tick(8.0 * 1000.0);
    assert!(m.cpu.pending_systick);
    assert_eq!(m.ppb_read(super::ppb::SYST_CALIB), 0x270f);
}

#[test]
fn reference_quirk_systick_does_not_wake_wfi() {
    // ARMv6-M: a pending SysTick wakes WFI. In rp2040js the SysTick alarm
    // pends the exception but nothing clears `waiting`; only a peripheral
    // IRQ line (setInterrupt) does. The Simulator loop keeps fast-forwarding.
    let mut m = machine_with_vectors(&[0xbf30, 0xbf00]);
    m.ppb_write(super::ppb::SYST_RVR, 99);
    m.ppb_write(super::ppb::SYST_CSR, 0b111);
    m.step(); // WFI
    for _ in 0..3 {
        m.step();
    }
    assert!(m.cpu.waiting);
    assert!(m.cpu.pending_systick);
    assert_eq!(m.cpu.ipsr, 0);
}

#[test]
fn step_loop_advances_the_clock_by_cycles_and_applies_inputs() {
    let mut m = machine(&[0x2001, 0x2002, 0xe7fe]); // MOVS; MOVS; B .
    m.inputs.push(ScheduledInput { at_cycle: 2.0, pin: 4, level: true });
    m.run(3);
    assert_eq!(m.clock.nanos, 8.0 * (1 + 1 + 2) as f64);
    assert_eq!(m.cpu.cycles, 4);
    assert!(m.sio.gpio[4].raw_input_value, "applied before the step starting at cycle 2");
    assert!(m.inputs.is_empty());
}

#[test]
fn idle_without_alarms_stops() {
    let mut m = machine(&[0xbf30]);
    m.run(10);
    assert_eq!(m.stop, Some(Stop::Idle));
    assert_eq!(m.steps, 1);
}
