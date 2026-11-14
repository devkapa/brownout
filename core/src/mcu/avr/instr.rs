//! The instruction set, ported from avr8js 0.21.0 `cpu/instruction.js` (MIT,
//! Copyright (c) Uri Shaked and contributors; see NOTICE).
//!
//! `decode` is the reference's `if / else if` chain in the same order (the
//! first match wins, so the order is part of the behaviour); `execute` is
//! its bodies. Cycle counts and flag arithmetic follow the reference
//! expression for expression, quirks included: where it departs from the
//! AVR instruction set manual the unit tests name it ("reference quirk"):
//!
//! - H (half carry) is bit 0 of the carry expression, not bit 3, in ADC, ADD,
//!   CP, CPC, CPI, NEG, SBC, SBCI, SBIW, SUB and SUBI;
//! - NEG never sets V (it compares the unwrapped result with 128);
//! - FMULSU sets Z when the product is NOT zero;
//! - CBI takes one cycle (the manual says two);
//! - SBC, SBCI and CPC lose Z when the byte result is 0 only because of the
//!   borrow (0 - 0xff - 1: the reference tests the unwrapped difference);
//! - SBIW writes H (the manual leaves it alone);
//! - an interrupt is taken on the instruction after SEI (the manual says two).

use super::cpu::{Cpu, ADDR_EIND, ADDR_RAMPZ, ADDR_SREG};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Adc, Add, Adiw, And, Andi, Asr, Bclr, Bld, Brbc, Brbs, Bset, Bst, Call, Cbi, Com, Cp, Cpc, Cpi,
    Cpse, Dec, Eicall, Eijmp, Elpm, ElpmReg, ElpmInc, Eor, Fmul, Fmuls, Fmulsu, Icall, Ijmp, In,
    Inc, Jmp, Lac, Las, Lat, Ldi, Lds, Ldx, LdxInc, LdxDec, Ldy, LdyInc, LdyDec, Lddy, Ldz, LdzInc,
    LdzDec, Lddz, Lpm, LpmReg, LpmInc, Lsr, Mov, Movw, Mul, Muls, Mulsu, Neg, Nop, Or, Sbr, Out,
    Pop, Push, Rcall, Ret, Reti, Rjmp, Ror, Sbc, Sbci, Sbi, Sbic, Sbis, Sbiw, Sbrc, Sbrs, Sleep,
    Spm, SpmInc, Sts, Stx, StxInc, StxDec, Sty, StyInc, StyDec, Stdy, Stz, StzInc, StzDec, Stdz,
    Sub, Subi, Swap, Wdr, Xch,
    /// No pattern matches: the reference does nothing but advance PC.
    Unknown,
}

impl Op {
    pub fn name(self) -> &'static str {
        match self {
            Op::Adc => "ADC", Op::Add => "ADD", Op::Adiw => "ADIW", Op::And => "AND",
            Op::Andi => "ANDI", Op::Asr => "ASR", Op::Bclr => "BCLR", Op::Bld => "BLD",
            Op::Brbc => "BRBC", Op::Brbs => "BRBS", Op::Bset => "BSET", Op::Bst => "BST",
            Op::Call => "CALL", Op::Cbi => "CBI", Op::Com => "COM", Op::Cp => "CP",
            Op::Cpc => "CPC", Op::Cpi => "CPI", Op::Cpse => "CPSE", Op::Dec => "DEC",
            Op::Eicall => "EICALL", Op::Eijmp => "EIJMP", Op::Elpm => "ELPM",
            Op::ElpmReg => "ELPM(REG)", Op::ElpmInc => "ELPM(INC)", Op::Eor => "EOR",
            Op::Fmul => "FMUL", Op::Fmuls => "FMULS", Op::Fmulsu => "FMULSU", Op::Icall => "ICALL",
            Op::Ijmp => "IJMP", Op::In => "IN", Op::Inc => "INC", Op::Jmp => "JMP",
            Op::Lac => "LAC", Op::Las => "LAS", Op::Lat => "LAT", Op::Ldi => "LDI",
            Op::Lds => "LDS", Op::Ldx => "LDX", Op::LdxInc => "LDX(INC)", Op::LdxDec => "LDX(DEC)",
            Op::Ldy => "LDY", Op::LdyInc => "LDY(INC)", Op::LdyDec => "LDY(DEC)",
            Op::Lddy => "LDDY", Op::Ldz => "LDZ", Op::LdzInc => "LDZ(INC)",
            Op::LdzDec => "LDZ(DEC)", Op::Lddz => "LDDZ", Op::Lpm => "LPM",
            Op::LpmReg => "LPM(REG)", Op::LpmInc => "LPM(INC)", Op::Lsr => "LSR", Op::Mov => "MOV",
            Op::Movw => "MOVW", Op::Mul => "MUL", Op::Muls => "MULS", Op::Mulsu => "MULSU",
            Op::Neg => "NEG", Op::Nop => "NOP", Op::Or => "OR", Op::Sbr => "SBR", Op::Out => "OUT",
            Op::Pop => "POP", Op::Push => "PUSH", Op::Rcall => "RCALL", Op::Ret => "RET",
            Op::Reti => "RETI", Op::Rjmp => "RJMP", Op::Ror => "ROR", Op::Sbc => "SBC",
            Op::Sbci => "SBCI", Op::Sbi => "SBI", Op::Sbic => "SBIC", Op::Sbis => "SBIS",
            Op::Sbiw => "SBIW", Op::Sbrc => "SBRC", Op::Sbrs => "SBRS", Op::Sleep => "SLEEP",
            Op::Spm => "SPM", Op::SpmInc => "SPM(INC)", Op::Sts => "STS", Op::Stx => "STX",
            Op::StxInc => "STX(INC)", Op::StxDec => "STX(DEC)", Op::Sty => "STY",
            Op::StyInc => "STY(INC)", Op::StyDec => "STY(DEC)", Op::Stdy => "STDY", Op::Stz => "STZ",
            Op::StzInc => "STZ(INC)", Op::StzDec => "STZ(DEC)", Op::Stdz => "STDZ", Op::Sub => "SUB",
            Op::Subi => "SUBI", Op::Swap => "SWAP", Op::Wdr => "WDR", Op::Xch => "XCH",
            Op::Unknown => "?",
        }
    }
}

/// The displacement field of LDD/STD: `q` is split across bits 13, 11-10, 2-0.
#[inline]
fn q_of(op: u32) -> u32 {
    (op & 7) | ((op & 0xc00) >> 7) | ((op & 0x2000) >> 8)
}

/// The reference's chain, in its order.
pub fn decode(opcode: u16) -> Op {
    let o = opcode as u32;
    if o & 0xfc00 == 0x1c00 { Op::Adc }
    else if o & 0xfc00 == 0x0c00 { Op::Add }
    else if o & 0xff00 == 0x9600 { Op::Adiw }
    else if o & 0xfc00 == 0x2000 { Op::And }
    else if o & 0xf000 == 0x7000 { Op::Andi }
    else if o & 0xfe0f == 0x9405 { Op::Asr }
    else if o & 0xff8f == 0x9488 { Op::Bclr }
    else if o & 0xfe08 == 0xf800 { Op::Bld }
    else if o & 0xfc00 == 0xf400 { Op::Brbc }
    else if o & 0xfc00 == 0xf000 { Op::Brbs }
    else if o & 0xff8f == 0x9408 { Op::Bset }
    else if o & 0xfe08 == 0xfa00 { Op::Bst }
    else if o & 0xfe0e == 0x940e { Op::Call }
    else if o & 0xff00 == 0x9800 { Op::Cbi }
    else if o & 0xfe0f == 0x9400 { Op::Com }
    else if o & 0xfc00 == 0x1400 { Op::Cp }
    else if o & 0xfc00 == 0x0400 { Op::Cpc }
    else if o & 0xf000 == 0x3000 { Op::Cpi }
    else if o & 0xfc00 == 0x1000 { Op::Cpse }
    else if o & 0xfe0f == 0x940a { Op::Dec }
    else if o == 0x9519 { Op::Eicall }
    else if o == 0x9419 { Op::Eijmp }
    else if o == 0x95d8 { Op::Elpm }
    else if o & 0xfe0f == 0x9006 { Op::ElpmReg }
    else if o & 0xfe0f == 0x9007 { Op::ElpmInc }
    else if o & 0xfc00 == 0x2400 { Op::Eor }
    else if o & 0xff88 == 0x0308 { Op::Fmul }
    else if o & 0xff88 == 0x0380 { Op::Fmuls }
    else if o & 0xff88 == 0x0388 { Op::Fmulsu }
    else if o == 0x9509 { Op::Icall }
    else if o == 0x9409 { Op::Ijmp }
    else if o & 0xf800 == 0xb000 { Op::In }
    else if o & 0xfe0f == 0x9403 { Op::Inc }
    else if o & 0xfe0e == 0x940c { Op::Jmp }
    else if o & 0xfe0f == 0x9206 { Op::Lac }
    else if o & 0xfe0f == 0x9205 { Op::Las }
    else if o & 0xfe0f == 0x9207 { Op::Lat }
    else if o & 0xf000 == 0xe000 { Op::Ldi }
    else if o & 0xfe0f == 0x9000 { Op::Lds }
    else if o & 0xfe0f == 0x900c { Op::Ldx }
    else if o & 0xfe0f == 0x900d { Op::LdxInc }
    else if o & 0xfe0f == 0x900e { Op::LdxDec }
    else if o & 0xfe0f == 0x8008 { Op::Ldy }
    else if o & 0xfe0f == 0x9009 { Op::LdyInc }
    else if o & 0xfe0f == 0x900a { Op::LdyDec }
    else if o & 0xd208 == 0x8008 && q_of(o) != 0 { Op::Lddy }
    else if o & 0xfe0f == 0x8000 { Op::Ldz }
    else if o & 0xfe0f == 0x9001 { Op::LdzInc }
    else if o & 0xfe0f == 0x9002 { Op::LdzDec }
    else if o & 0xd208 == 0x8000 && q_of(o) != 0 { Op::Lddz }
    else if o == 0x95c8 { Op::Lpm }
    else if o & 0xfe0f == 0x9004 { Op::LpmReg }
    else if o & 0xfe0f == 0x9005 { Op::LpmInc }
    else if o & 0xfe0f == 0x9406 { Op::Lsr }
    else if o & 0xfc00 == 0x2c00 { Op::Mov }
    else if o & 0xff00 == 0x0100 { Op::Movw }
    else if o & 0xfc00 == 0x9c00 { Op::Mul }
    else if o & 0xff00 == 0x0200 { Op::Muls }
    else if o & 0xff88 == 0x0300 { Op::Mulsu }
    else if o & 0xfe0f == 0x9401 { Op::Neg }
    else if o == 0 { Op::Nop }
    else if o & 0xfc00 == 0x2800 { Op::Or }
    else if o & 0xf000 == 0x6000 { Op::Sbr }
    else if o & 0xf800 == 0xb800 { Op::Out }
    else if o & 0xfe0f == 0x900f { Op::Pop }
    else if o & 0xfe0f == 0x920f { Op::Push }
    else if o & 0xf000 == 0xd000 { Op::Rcall }
    else if o == 0x9508 { Op::Ret }
    else if o == 0x9518 { Op::Reti }
    else if o & 0xf000 == 0xc000 { Op::Rjmp }
    else if o & 0xfe0f == 0x9407 { Op::Ror }
    else if o & 0xfc00 == 0x0800 { Op::Sbc }
    else if o & 0xf000 == 0x4000 { Op::Sbci }
    else if o & 0xff00 == 0x9a00 { Op::Sbi }
    else if o & 0xff00 == 0x9900 { Op::Sbic }
    else if o & 0xff00 == 0x9b00 { Op::Sbis }
    else if o & 0xff00 == 0x9700 { Op::Sbiw }
    else if o & 0xfe08 == 0xfc00 { Op::Sbrc }
    else if o & 0xfe08 == 0xfe00 { Op::Sbrs }
    else if o == 0x9588 { Op::Sleep }
    else if o == 0x95e8 { Op::Spm }
    else if o == 0x95f8 { Op::SpmInc }
    else if o & 0xfe0f == 0x9200 { Op::Sts }
    else if o & 0xfe0f == 0x920c { Op::Stx }
    else if o & 0xfe0f == 0x920d { Op::StxInc }
    else if o & 0xfe0f == 0x920e { Op::StxDec }
    else if o & 0xfe0f == 0x8208 { Op::Sty }
    else if o & 0xfe0f == 0x9209 { Op::StyInc }
    else if o & 0xfe0f == 0x920a { Op::StyDec }
    else if o & 0xd208 == 0x8208 && q_of(o) != 0 { Op::Stdy }
    else if o & 0xfe0f == 0x8200 { Op::Stz }
    else if o & 0xfe0f == 0x9201 { Op::StzInc }
    else if o & 0xfe0f == 0x9202 { Op::StzDec }
    else if o & 0xd208 == 0x8200 && q_of(o) != 0 { Op::Stdz }
    else if o & 0xfc00 == 0x1800 { Op::Sub }
    else if o & 0xf000 == 0x5000 { Op::Subi }
    else if o & 0xfe0f == 0x9402 { Op::Swap }
    else if o == 0x95a8 { Op::Wdr }
    else if o & 0xfe0f == 0x9204 { Op::Xch }
    else { Op::Unknown }
}

/// `isTwoWordInstruction`: LDS, STS, CALL, JMP. `None` is a program word the
/// flash does not have.
fn is_two_word(next: Option<u16>) -> bool {
    match next {
        Some(w) => {
            let w = w as u32;
            w & 0xfe0f == 0x9000 || w & 0xfe0f == 0x9200 || w & 0xfe0e == 0x940e || w & 0xfe0e == 0x940c
        }
        None => false,
    }
}

#[inline]
fn dd(o: u32) -> usize {
    ((o & 0x1f0) >> 4) as usize
}
#[inline]
fn rr(o: u32) -> usize {
    ((o & 0xf) | ((o & 0x200) >> 5)) as usize
}
#[inline]
fn bit(b: i32, on: u8) -> u8 {
    if b != 0 { on } else { 0 }
}
/// `((sreg >> 2) & 1) ^ ((sreg >> 3) & 1) ? 0x10 : 0`: S = N xor V.
#[inline]
fn s_flag(sreg: u8) -> u8 {
    if ((sreg >> 2) & 1) ^ ((sreg >> 3) & 1) != 0 { 0x10 } else { 0 }
}

/// Reads the 22-bit-aware return address pushed by CALL-class instructions.
fn push_return(cpu: &mut Cpu, ret: i32, always_three: bool) {
    let sp = cpu.sp() as i32;
    cpu.wr(sp, (ret & 255) as u8);
    cpu.wr(sp - 1, ((ret >> 8) & 255) as u8);
    let three = always_three || cpu.pc22_bits;
    if three {
        cpu.wr(sp - 2, ((ret >> 16) & 255) as u8);
    }
    cpu.set_sp((sp - if three { 3 } else { 2 }) as u16);
}

fn skip(cpu: &mut Cpu) {
    let size = if is_two_word(cpu.word(cpu.pc + 1)) { 2 } else { 1 };
    cpu.pc += size;
    cpu.cycles += size as u64;
}

/// `avrInstruction`: executes the instruction at PC. The caller then calls
/// `Cpu::tick`, as `ArduinoMcu.step` does after every instruction.
pub fn avr_instruction(cpu: &mut Cpu) {
    let opcode = cpu.word(cpu.pc);
    let op = match opcode {
        Some(w) => decode(w),
        None => Op::Unknown,
    };
    let o = opcode.unwrap_or(0) as u32;
    let sr = ADDR_SREG;
    match op {
        Op::Adc => {
            let d = cpu.data[dd(o)] as i32;
            let r = cpu.data[rr(o)] as i32;
            let sum = d + r + (cpu.data[sr] & 1) as i32;
            let res = sum & 255;
            cpu.data[dd(o)] = res as u8;
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= bit((res ^ r) & (d ^ res) & 128, 8);
            s |= s_flag(s);
            s |= bit(sum & 256, 1);
            s |= bit(1 & ((d & r) | (r & !res) | (!res & d)), 0x20);
            cpu.data[sr] = s;
        }
        Op::Add => {
            let d = cpu.data[dd(o)] as i32;
            let r = cpu.data[rr(o)] as i32;
            let res = (d + r) & 255;
            cpu.data[dd(o)] = res as u8;
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= bit((res ^ r) & (res ^ d) & 128, 8);
            s |= s_flag(s);
            s |= bit((d + r) & 256, 1);
            s |= bit(1 & ((d & r) | (r & !res) | (!res & d)), 0x20);
            cpu.data[sr] = s;
        }
        Op::Adiw => {
            let addr = (2 * ((o & 0x30) >> 4) + 24) as usize;
            let value = cpu.get16(addr) as i32;
            let res = (value + ((o & 0xf) | ((o & 0xc0) >> 2)) as i32) & 0xffff;
            cpu.set16(addr, res as u16);
            let mut s = cpu.data[sr] & 0xe0;
            s |= bit((res == 0) as i32, 2);
            s |= bit(0x8000 & res, 4);
            s |= bit(!value & res & 0x8000, 8);
            s |= s_flag(s);
            s |= bit(!res & value & 0x8000, 1);
            cpu.data[sr] = s;
            cpu.cycles += 1;
        }
        Op::And => {
            let res = cpu.data[dd(o)] & cpu.data[rr(o)];
            cpu.data[dd(o)] = res;
            logic_flags(cpu, res);
        }
        Op::Andi => {
            let d = ((o & 0xf0) >> 4) as usize + 16;
            let res = cpu.data[d] & (((o & 0xf) | ((o & 0xf00) >> 4)) as u8);
            cpu.data[d] = res;
            logic_flags(cpu, res);
        }
        Op::Asr => {
            let value = cpu.data[dd(o)] as i32;
            let res = (value >> 1) | (128 & value);
            cpu.data[dd(o)] = res as u8;
            let mut s = cpu.data[sr] & 0xe0;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= (value & 1) as u8;
            s |= bit((((s >> 2) & 1) ^ (s & 1)) as i32, 8);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Bclr => {
            cpu.data[sr] &= !(1u8 << ((o & 0x70) >> 4));
        }
        Op::Bld => {
            let b = o & 7;
            let d = dd(o);
            cpu.data[d] = (!(1u8 << b) & cpu.data[d]) | (((cpu.data[sr] >> 6) & 1) << b);
        }
        Op::Brbc => {
            if cpu.data[sr] & (1 << (o & 7)) == 0 {
                cpu.pc += (((o & 0x1f8) >> 3) as i32) - if o & 0x200 != 0 { 0x40 } else { 0 };
                cpu.cycles += 1;
            }
        }
        Op::Brbs => {
            if cpu.data[sr] & (1 << (o & 7)) != 0 {
                cpu.pc += (((o & 0x1f8) >> 3) as i32) - if o & 0x200 != 0 { 0x40 } else { 0 };
                cpu.cycles += 1;
            }
        }
        Op::Bset => {
            cpu.data[sr] |= 1u8 << ((o & 0x70) >> 4);
        }
        Op::Bst => {
            let d = cpu.data[dd(o)];
            let b = o & 7;
            cpu.data[sr] = (cpu.data[sr] & 0xbf) | if (d >> b) & 1 != 0 { 0x40 } else { 0 };
        }
        Op::Call => {
            let k = cpu.word(cpu.pc + 1).unwrap_or(0) as i32 | (((o & 1) << 16) as i32) | (((o & 0x1f0) << 13) as i32);
            let ret = cpu.pc + 2;
            push_return(cpu, ret, false);
            cpu.pc = k - 1;
            cpu.cycles += if cpu.pc22_bits { 4 } else { 3 };
        }
        Op::Cbi => {
            let a = (o & 0xf8) as i32;
            let b = o & 7;
            let r = cpu.read_data((a >> 3) + 32);
            let mask = 1u8 << b;
            cpu.write_data((a >> 3) + 32, r & !mask);
        }
        Op::Com => {
            let d = dd(o);
            let res = 255 - cpu.data[d] as i32;
            cpu.data[d] = res as u8;
            let mut s = (cpu.data[sr] & 0xe1) | 1;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Cp => {
            let v1 = cpu.data[dd(o)] as i32;
            let v2 = cpu.data[rr(o)] as i32;
            let res = v1 - v2;
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= bit((v1 ^ v2) & (v1 ^ res) & 128, 8);
            s |= s_flag(s);
            s |= bit((v2 > v1) as i32, 1);
            s |= bit(1 & ((!v1 & v2) | (v2 & res) | (res & !v1)), 0x20);
            cpu.data[sr] = s;
        }
        Op::Cpc => {
            let a1 = cpu.data[dd(o)] as i32;
            let a2 = cpu.data[rr(o)] as i32;
            let old = cpu.data[sr];
            let r = a1 - a2 - (old & 1) as i32;
            let mut s = (old & 0xc0)
                | bit((r == 0 && (old >> 1) & 1 != 0) as i32, 2)
                | bit((a2 + (old & 1) as i32 > a1) as i32, 1);
            s |= bit(128 & r, 4);
            s |= bit((a1 ^ a2) & (a1 ^ r) & 128, 8);
            s |= s_flag(s);
            s |= bit(1 & ((!a1 & a2) | (a2 & r) | (r & !a1)), 0x20);
            cpu.data[sr] = s;
        }
        Op::Cpi => {
            let a1 = cpu.data[((o & 0xf0) >> 4) as usize + 16] as i32;
            let a2 = ((o & 0xf) | ((o & 0xf00) >> 4)) as i32;
            let r = a1 - a2;
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((r == 0) as i32, 2);
            s |= bit(128 & r, 4);
            s |= bit((a1 ^ a2) & (a1 ^ r) & 128, 8);
            s |= s_flag(s);
            s |= bit((a2 > a1) as i32, 1);
            s |= bit(1 & ((!a1 & a2) | (a2 & r) | (r & !a1)), 0x20);
            cpu.data[sr] = s;
        }
        Op::Cpse => {
            if cpu.data[dd(o)] == cpu.data[rr(o)] {
                skip(cpu);
            }
        }
        Op::Dec => {
            let value = cpu.data[dd(o)] as i32;
            let res = value - 1;
            cpu.data[dd(o)] = res as u8;
            let mut s = cpu.data[sr] & 0xe1;
            s |= bit((res == 0) as i32, 2);
            s |= bit(128 & res, 4);
            s |= bit((value == 128) as i32, 8);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Eicall => {
            let ret = cpu.pc + 1;
            let eind = cpu.data[ADDR_EIND] as i32;
            push_return(cpu, ret, true);
            cpu.pc = ((eind << 16) | cpu.get16(30) as i32) - 1;
            cpu.cycles += 3;
        }
        Op::Eijmp => {
            let eind = cpu.data[ADDR_EIND] as i32;
            cpu.pc = ((eind << 16) | cpu.get16(30) as i32) - 1;
            cpu.cycles += 1;
        }
        Op::Elpm => {
            let rampz = cpu.data[ADDR_RAMPZ] as u32;
            cpu.data[0] = cpu.prog_byte((rampz << 16) | cpu.get16(30) as u32);
            cpu.cycles += 2;
        }
        Op::ElpmReg => {
            let rampz = cpu.data[ADDR_RAMPZ] as u32;
            cpu.data[dd(o)] = cpu.prog_byte((rampz << 16) | cpu.get16(30) as u32);
            cpu.cycles += 2;
        }
        Op::ElpmInc => {
            let rampz = cpu.data[ADDR_RAMPZ] as u32;
            let i = cpu.get16(30);
            cpu.data[dd(o)] = cpu.prog_byte((rampz << 16) | i as u32);
            cpu.set16(30, i.wrapping_add(1));
            if i == 0xffff {
                let banks = (cpu.prog_len_bytes() >> 16) as u32;
                // `% 0` is NaN in the reference, which a Uint8Array stores as 0.
                cpu.data[ADDR_RAMPZ] = if banks == 0 { 0 } else { ((rampz + 1) % banks) as u8 };
            }
            cpu.cycles += 2;
        }
        Op::Eor => {
            let res = cpu.data[dd(o)] ^ cpu.data[rr(o)];
            cpu.data[dd(o)] = res;
            logic_flags(cpu, res);
        }
        Op::Fmul => {
            let v1 = cpu.data[((o & 0x70) >> 4) as usize + 16] as i32;
            let v2 = cpu.data[(o & 7) as usize + 16] as i32;
            let r = (v1 * v2) << 1;
            cpu.set16(0, r as u16);
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r == 0) as i32, 2) | bit((v1 * v2) & 0x8000, 1);
            cpu.cycles += 1;
        }
        Op::Fmuls => {
            let v1 = cpu.data[((o & 0x70) >> 4) as usize + 16] as i8 as i32;
            let v2 = cpu.data[(o & 7) as usize + 16] as i8 as i32;
            let r = (v1 * v2) << 1;
            cpu.set16(0, r as u16);
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r == 0) as i32, 2) | bit((v1 * v2) & 0x8000, 1);
            cpu.cycles += 1;
        }
        Op::Fmulsu => {
            let v1 = cpu.data[((o & 0x70) >> 4) as usize + 16] as i8 as i32;
            let v2 = cpu.data[(o & 7) as usize + 16] as i32;
            let r = (v1 * v2) << 1;
            cpu.set16(0, r as u16);
            // Reference quirk: Z is set when the product is NOT zero.
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r != 0) as i32, 2) | bit((v1 * v2) & 0x8000, 1);
            cpu.cycles += 1;
        }
        Op::Icall => {
            let ret = cpu.pc + 1;
            push_return(cpu, ret, false);
            cpu.pc = cpu.get16(30) as i32 - 1;
            cpu.cycles += if cpu.pc22_bits { 3 } else { 2 };
        }
        Op::Ijmp => {
            cpu.pc = cpu.get16(30) as i32 - 1;
            cpu.cycles += 1;
        }
        Op::In => {
            let a = ((o & 0xf) | ((o & 0x600) >> 5)) as i32 + 32;
            let v = cpu.read_data(a);
            cpu.data[dd(o)] = v;
        }
        Op::Inc => {
            let d = cpu.data[dd(o)] as i32;
            let r = (d + 1) & 255;
            cpu.data[dd(o)] = r as u8;
            let mut s = cpu.data[sr] & 0xe1;
            s |= bit((r == 0) as i32, 2);
            s |= bit(128 & r, 4);
            s |= bit((d == 127) as i32, 8);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Jmp => {
            let k = cpu.word(cpu.pc + 1).unwrap_or(0) as i32 | (((o & 1) << 16) as i32) | (((o & 0x1f0) << 13) as i32);
            cpu.pc = k - 1;
            cpu.cycles += 2;
        }
        Op::Lac => {
            let r = dd(o);
            let clear = cpu.data[r];
            let z = cpu.get16(30) as i32;
            let value = cpu.read_data(z);
            cpu.write_data(z, value & (255 - clear));
            cpu.data[r] = value;
        }
        Op::Las => {
            let r = dd(o);
            let set = cpu.data[r];
            let z = cpu.get16(30) as i32;
            let value = cpu.read_data(z);
            cpu.write_data(z, value | set);
            cpu.data[r] = value;
        }
        Op::Lat => {
            let r = cpu.data[dd(o)];
            let z = cpu.get16(30) as i32;
            let v = cpu.read_data(z);
            cpu.write_data(z, r ^ v);
            cpu.data[dd(o)] = v;
        }
        Op::Ldi => {
            cpu.data[((o & 0xf0) >> 4) as usize + 16] = ((o & 0xf) | ((o & 0xf00) >> 4)) as u8;
        }
        Op::Lds => {
            cpu.cycles += 1;
            let addr = cpu.word(cpu.pc + 1).map(|w| w as i32);
            let value = match addr {
                Some(a) => cpu.read_data(a),
                None => 0,
            };
            cpu.data[dd(o)] = value;
            cpu.pc += 1;
        }
        Op::Ldx => {
            cpu.cycles += 1;
            let v = cpu.read_data(cpu.get16(26) as i32);
            cpu.data[dd(o)] = v;
        }
        Op::LdxInc => {
            let x = cpu.get16(26);
            cpu.cycles += 1;
            let v = cpu.read_data(x as i32);
            cpu.data[dd(o)] = v;
            cpu.set16(26, x.wrapping_add(1));
        }
        Op::LdxDec => {
            let x = cpu.get16(26) as i32 - 1;
            cpu.set16(26, x as u16);
            cpu.cycles += 1;
            let v = cpu.read_data(x);
            cpu.data[dd(o)] = v;
        }
        Op::Ldy => {
            cpu.cycles += 1;
            let v = cpu.read_data(cpu.get16(28) as i32);
            cpu.data[dd(o)] = v;
        }
        Op::LdyInc => {
            let y = cpu.get16(28);
            cpu.cycles += 1;
            let v = cpu.read_data(y as i32);
            cpu.data[dd(o)] = v;
            cpu.set16(28, y.wrapping_add(1));
        }
        Op::LdyDec => {
            let y = cpu.get16(28) as i32 - 1;
            cpu.set16(28, y as u16);
            cpu.cycles += 1;
            let v = cpu.read_data(y);
            cpu.data[dd(o)] = v;
        }
        Op::Lddy => {
            cpu.cycles += 1;
            let v = cpu.read_data(cpu.get16(28) as i32 + q_of(o) as i32);
            cpu.data[dd(o)] = v;
        }
        Op::Ldz => {
            cpu.cycles += 1;
            let v = cpu.read_data(cpu.get16(30) as i32);
            cpu.data[dd(o)] = v;
        }
        Op::LdzInc => {
            let z = cpu.get16(30);
            cpu.cycles += 1;
            let v = cpu.read_data(z as i32);
            cpu.data[dd(o)] = v;
            cpu.set16(30, z.wrapping_add(1));
        }
        Op::LdzDec => {
            let z = cpu.get16(30) as i32 - 1;
            cpu.set16(30, z as u16);
            cpu.cycles += 1;
            let v = cpu.read_data(z);
            cpu.data[dd(o)] = v;
        }
        Op::Lddz => {
            cpu.cycles += 1;
            let v = cpu.read_data(cpu.get16(30) as i32 + q_of(o) as i32);
            cpu.data[dd(o)] = v;
        }
        Op::Lpm => {
            cpu.data[0] = cpu.prog_byte(cpu.get16(30) as u32);
            cpu.cycles += 2;
        }
        Op::LpmReg => {
            cpu.data[dd(o)] = cpu.prog_byte(cpu.get16(30) as u32);
            cpu.cycles += 2;
        }
        Op::LpmInc => {
            let i = cpu.get16(30);
            cpu.data[dd(o)] = cpu.prog_byte(i as u32);
            cpu.set16(30, i.wrapping_add(1));
            cpu.cycles += 2;
        }
        Op::Lsr => {
            let value = cpu.data[dd(o)] as i32;
            let res = value >> 1;
            cpu.data[dd(o)] = res as u8;
            let mut s = cpu.data[sr] & 0xe0;
            s |= bit((res == 0) as i32, 2);
            s |= (value & 1) as u8;
            s |= bit((((s >> 2) & 1) ^ (s & 1)) as i32, 8);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Mov => {
            cpu.data[dd(o)] = cpu.data[rr(o)];
        }
        Op::Movw => {
            let r2 = 2 * (o & 0xf) as usize;
            let d2 = 2 * ((o & 0xf0) >> 4) as usize;
            cpu.data[d2] = cpu.data[r2];
            cpu.data[d2 + 1] = cpu.data[r2 + 1];
        }
        Op::Mul => {
            let r = cpu.data[dd(o)] as i32 * cpu.data[rr(o)] as i32;
            cpu.set16(0, r as u16);
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r == 0) as i32, 2) | bit(0x8000 & r, 1);
            cpu.cycles += 1;
        }
        Op::Muls => {
            let r = (cpu.data[((o & 0xf0) >> 4) as usize + 16] as i8 as i32) * (cpu.data[(o & 0xf) as usize + 16] as i8 as i32);
            cpu.set16(0, r as u16);
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r == 0) as i32, 2) | bit(0x8000 & r, 1);
            cpu.cycles += 1;
        }
        Op::Mulsu => {
            let r = (cpu.data[((o & 0x70) >> 4) as usize + 16] as i8 as i32) * (cpu.data[(o & 7) as usize + 16] as i32);
            cpu.set16(0, r as u16);
            cpu.data[sr] = (cpu.data[sr] & 0xfc) | bit((0xffff & r == 0) as i32, 2) | bit(0x8000 & r, 1);
            cpu.cycles += 1;
        }
        Op::Neg => {
            let d = dd(o);
            let value = cpu.data[d] as i32;
            let r = 0 - value;
            cpu.data[d] = r as u8;
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((r == 0) as i32, 2);
            s |= bit(128 & r, 4);
            // Reference quirk: `128 === R` on the unwrapped (<= 0) result, so
            // V is never set.
            s |= bit((r == 128) as i32, 8);
            s |= s_flag(s);
            s |= bit((r != 0) as i32, 1);
            s |= bit(1 & (r | value), 0x20);
            cpu.data[sr] = s;
        }
        Op::Nop => {}
        Op::Or => {
            let res = cpu.data[dd(o)] | cpu.data[rr(o)];
            cpu.data[dd(o)] = res;
            logic_flags(cpu, res);
        }
        Op::Sbr => {
            let d = ((o & 0xf0) >> 4) as usize + 16;
            let res = cpu.data[d] | (((o & 0xf) | ((o & 0xf00) >> 4)) as u8);
            cpu.data[d] = res;
            logic_flags(cpu, res);
        }
        Op::Out => {
            let a = ((o & 0xf) | ((o & 0x600) >> 5)) as i32 + 32;
            let v = cpu.data[dd(o)];
            cpu.write_data(a, v);
        }
        Op::Pop => {
            let value = cpu.sp() as i32 + 1;
            cpu.set_sp(value as u16);
            cpu.data[dd(o)] = cpu.rd(value);
            cpu.cycles += 1;
        }
        Op::Push => {
            let value = cpu.sp();
            cpu.wr(value as i32, cpu.data[dd(o)]);
            cpu.set_sp(value.wrapping_sub(1));
            cpu.cycles += 1;
        }
        Op::Rcall => {
            let k = (o & 0x7ff) as i32 - if o & 0x800 != 0 { 0x800 } else { 0 };
            let ret = cpu.pc + 1;
            push_return(cpu, ret, false);
            cpu.pc += k;
            cpu.cycles += if cpu.pc22_bits { 3 } else { 2 };
        }
        Op::Ret | Op::Reti => {
            let n = if cpu.pc22_bits { 3 } else { 2 };
            let i = cpu.sp() as i32 + n;
            cpu.set_sp(i as u16);
            cpu.pc = ((cpu.rd(i - 1) as i32) << 8) + cpu.rd(i) as i32 - 1;
            if cpu.pc22_bits {
                cpu.pc |= (cpu.rd(i - 2) as i32) << 16;
            }
            cpu.cycles += if cpu.pc22_bits { 4 } else { 3 };
            if op == Op::Reti {
                cpu.data[sr] |= 0x80;
            }
        }
        Op::Rjmp => {
            cpu.pc += (o & 0x7ff) as i32 - if o & 0x800 != 0 { 0x800 } else { 0 };
            cpu.cycles += 1;
        }
        Op::Ror => {
            let d = cpu.data[dd(o)] as i32;
            let r = (d >> 1) | (((cpu.data[sr] & 1) as i32) << 7);
            cpu.data[dd(o)] = r as u8;
            let mut s = cpu.data[sr] & 0xe0;
            s |= bit((r == 0) as i32, 2);
            s |= bit(128 & r, 4);
            s |= (1 & d) as u8;
            s |= bit((((s >> 2) & 1) ^ (s & 1)) as i32, 8);
            s |= s_flag(s);
            cpu.data[sr] = s;
        }
        Op::Sbc => {
            let v1 = cpu.data[dd(o)] as i32;
            let v2 = cpu.data[rr(o)] as i32;
            sbc_like(cpu, dd(o), v1, v2);
        }
        Op::Sbci => {
            let d = ((o & 0xf0) >> 4) as usize + 16;
            let v1 = cpu.data[d] as i32;
            let v2 = ((o & 0xf) | ((o & 0xf00) >> 4)) as i32;
            sbc_like(cpu, d, v1, v2);
        }
        Op::Sbi => {
            let target = (((o & 0xf8) >> 3) + 32) as i32;
            let mask = 1u8 << (o & 7);
            let v = cpu.read_data(target) | mask;
            cpu.write_data(target, v);
            cpu.cycles += 1;
        }
        Op::Sbic => {
            let value = cpu.read_data((((o & 0xf8) >> 3) + 32) as i32);
            if value & (1 << (o & 7)) == 0 {
                skip(cpu);
            }
        }
        Op::Sbis => {
            let value = cpu.read_data((((o & 0xf8) >> 3) + 32) as i32);
            if value & (1 << (o & 7)) != 0 {
                skip(cpu);
            }
        }
        Op::Sbiw => {
            let i = (2 * ((o & 0x30) >> 4) + 24) as usize;
            let a = cpu.get16(i) as i32;
            let l = ((o & 0xf) | ((o & 0xc0) >> 2)) as i32;
            let r = a - l;
            cpu.set16(i, r as u16);
            let mut s = cpu.data[sr] & 0xc0;
            s |= bit((r == 0) as i32, 2);
            s |= bit(0x8000 & r, 4);
            s |= bit(a & !r & 0x8000, 8);
            s |= s_flag(s);
            s |= bit((l > a) as i32, 1);
            s |= bit(1 & ((!a & l) | (l & r) | (r & !a)), 0x20);
            cpu.data[sr] = s;
            cpu.cycles += 1;
        }
        Op::Sbrc => {
            if cpu.data[dd(o)] & (1 << (o & 7)) == 0 {
                skip(cpu);
            }
        }
        Op::Sbrs => {
            if cpu.data[dd(o)] & (1 << (o & 7)) != 0 {
                skip(cpu);
            }
        }
        Op::Sleep | Op::Spm | Op::SpmInc => {}
        Op::Sts => {
            let v = cpu.data[dd(o)];
            if let Some(a) = cpu.word(cpu.pc + 1) {
                cpu.write_data(a as i32, v);
            }
            cpu.pc += 1;
            cpu.cycles += 1;
        }
        Op::Stx => {
            let v = cpu.data[dd(o)];
            cpu.write_data(cpu.get16(26) as i32, v);
            cpu.cycles += 1;
        }
        Op::StxInc => {
            let x = cpu.get16(26);
            let v = cpu.data[dd(o)];
            cpu.write_data(x as i32, v);
            cpu.set16(26, x.wrapping_add(1));
            cpu.cycles += 1;
        }
        Op::StxDec => {
            let v = cpu.data[dd(o)];
            let x = cpu.get16(26) as i32 - 1;
            cpu.set16(26, x as u16);
            cpu.write_data(x, v);
            cpu.cycles += 1;
        }
        Op::Sty => {
            let v = cpu.data[dd(o)];
            cpu.write_data(cpu.get16(28) as i32, v);
            cpu.cycles += 1;
        }
        Op::StyInc => {
            let v = cpu.data[dd(o)];
            let y = cpu.get16(28);
            cpu.write_data(y as i32, v);
            cpu.set16(28, y.wrapping_add(1));
            cpu.cycles += 1;
        }
        Op::StyDec => {
            let v = cpu.data[dd(o)];
            let y = cpu.get16(28) as i32 - 1;
            cpu.set16(28, y as u16);
            cpu.write_data(y, v);
            cpu.cycles += 1;
        }
        Op::Stdy => {
            let v = cpu.data[dd(o)];
            cpu.write_data(cpu.get16(28) as i32 + q_of(o) as i32, v);
            cpu.cycles += 1;
        }
        Op::Stz => {
            let v = cpu.data[dd(o)];
            cpu.write_data(cpu.get16(30) as i32, v);
            cpu.cycles += 1;
        }
        Op::StzInc => {
            let z = cpu.get16(30);
            let v = cpu.data[dd(o)];
            cpu.write_data(z as i32, v);
            cpu.set16(30, z.wrapping_add(1));
            cpu.cycles += 1;
        }
        Op::StzDec => {
            let v = cpu.data[dd(o)];
            let z = cpu.get16(30) as i32 - 1;
            cpu.set16(30, z as u16);
            cpu.write_data(z, v);
            cpu.cycles += 1;
        }
        Op::Stdz => {
            let v = cpu.data[dd(o)];
            cpu.write_data(cpu.get16(30) as i32 + q_of(o) as i32, v);
            cpu.cycles += 1;
        }
        Op::Sub => {
            let v1 = cpu.data[dd(o)] as i32;
            let v2 = cpu.data[rr(o)] as i32;
            sub_like(cpu, dd(o), v1, v2);
        }
        Op::Subi => {
            let d = ((o & 0xf0) >> 4) as usize + 16;
            let v1 = cpu.data[d] as i32;
            let v2 = ((o & 0xf) | ((o & 0xf00) >> 4)) as i32;
            sub_like(cpu, d, v1, v2);
        }
        Op::Swap => {
            let d = dd(o);
            let i = cpu.data[d];
            cpu.data[d] = ((15 & i) << 4) | ((240 & i) >> 4);
        }
        Op::Wdr => {
            cpu.watchdog_resets += 1;
        }
        Op::Xch => {
            // Direct data[] access, as in the reference (no peripheral hook).
            let r = dd(o);
            let z = cpu.get16(30) as i32;
            let v1 = cpu.data[r];
            let v2 = cpu.rd(z);
            cpu.wr(z, v1);
            cpu.data[r] = v2;
        }
        Op::Unknown => {}
    }
    let len = cpu.prog.len() as i32;
    cpu.pc = (cpu.pc + 1) % len;
    cpu.cycles += 1;
}

/// The flags of AND/ANDI/EOR/OR/SBR: Z, N, S; V cleared, C and H kept.
fn logic_flags(cpu: &mut Cpu, res: u8) {
    let mut s = cpu.data[ADDR_SREG] & 0xe1;
    s |= if res == 0 { 2 } else { 0 };
    s |= if res & 128 != 0 { 4 } else { 0 };
    s |= s_flag(s);
    cpu.data[ADDR_SREG] = s;
}

/// SUB and SUBI.
fn sub_like(cpu: &mut Cpu, d: usize, v1: i32, v2: i32) {
    let r = v1 - v2;
    cpu.data[d] = r as u8;
    let mut s = cpu.data[ADDR_SREG] & 0xc0;
    s |= bit((r == 0) as i32, 2);
    s |= bit(128 & r, 4);
    s |= bit((v1 ^ v2) & (v1 ^ r) & 128, 8);
    s |= s_flag(s);
    s |= bit((v2 > v1) as i32, 1);
    s |= bit(1 & ((!v1 & v2) | (v2 & r) | (r & !v1)), 0x20);
    cpu.data[ADDR_SREG] = s;
}

/// SBC and SBCI: Z is only kept if it was set.
fn sbc_like(cpu: &mut Cpu, d: usize, v1: i32, v2: i32) {
    let old = cpu.data[ADDR_SREG];
    let r = v1 - v2 - (old & 1) as i32;
    cpu.data[d] = r as u8;
    let mut s = (old & 0xc0)
        | bit((r == 0 && (old >> 1) & 1 != 0) as i32, 2)
        | bit((v2 + (old & 1) as i32 > v1) as i32, 1);
    s |= bit(128 & r, 4);
    s |= bit((v1 ^ v2) & (v1 ^ r) & 128, 8);
    s |= s_flag(s);
    s |= bit(1 & ((!v1 & v2) | (v2 & r) | (r & !v1)), 0x20);
    cpu.data[ADDR_SREG] = s;
}
