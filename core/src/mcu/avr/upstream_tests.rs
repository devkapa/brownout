//! avr8js 0.21.0's CPU, instruction and interrupt suites, ported to the Rust
//! port (MIT, Copyright (c) Uri Shaked and contributors; see NOTICE).
//! Upstream files (github.com/wokwi/avr8js, tag v0.21.0, 898f76c):
//! `src/cpu/instruction.spec.ts` (97 tests), `src/cpu/cpu.spec.ts` (4),
//! `src/cpu/interrupt.spec.ts` (2).
//!
//! The instruction tests were converted from the TypeScript statement by
//! statement (a one-off script that only read the spec text; nothing from the
//! upstream checkout was run) and the output reviewed; each test keeps its
//! upstream title in the doc comment. Upstream builds programs with
//! `utils/assembler.ts`; `asm` below is a small assembler for exactly the
//! mnemonics the suite uses, written from the AVR instruction set manual's
//! encodings and independent of the port's decoder. `cpu.spec` and
//! `interrupt.spec` are written out by hand, because the port keys clock
//! events by id instead of by callback.

#![allow(unused_mut)]

use super::cpu::Cpu;
use super::instr::avr_instruction;
use super::interrupt::avr_interrupt;

#[allow(non_upper_case_globals, dead_code)]
mod names {
    pub const r0: usize = 0;
    pub const r1: usize = 1;
    pub const r2: usize = 2;
    pub const r3: usize = 3;
    pub const r4: usize = 4;
    pub const r5: usize = 5;
    pub const r6: usize = 6;
    pub const r7: usize = 7;
    pub const r8: usize = 8;
    pub const r16: usize = 16;
    pub const r17: usize = 17;
    pub const r18: usize = 18;
    pub const r19: usize = 19;
    pub const r20: usize = 20;
    pub const r21: usize = 21;
    pub const r22: usize = 22;
    pub const r23: usize = 23;
    pub const r24: usize = 24;
    pub const r26: usize = 26;
    pub const r27: usize = 27;
    pub const r31: usize = 31;
    pub const X: usize = 26;
    pub const Y: usize = 28;
    pub const Z: usize = 30;
    pub const RAMPZ: usize = 0x5b;
    pub const EIND: usize = 0x5c;
    pub const SP: usize = 93;
    pub const SPH: usize = 94;
    pub const SREG: usize = 95;
    pub const SREG_C: u8 = 0b00000001;
    pub const SREG_Z: u8 = 0b00000010;
    pub const SREG_N: u8 = 0b00000100;
    pub const SREG_V: u8 = 0b00001000;
    pub const SREG_S: u8 = 0b00010000;
    pub const SREG_H: u8 = 0b00100000;
    pub const SREG_I: u8 = 0b10000000;
}
use names::*;

// ---- a small assembler (manual encodings) ------------------------------------

fn num(s: &str) -> i32 {
    let s = s.trim().trim_start_matches('+');
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let v = if let Some(h) = s.strip_prefix("0x") { i32::from_str_radix(h, 16).unwrap() } else { s.parse().unwrap() };
    if neg { -v } else { v }
}

fn reg(s: &str) -> u16 {
    let n: u16 = s.trim().trim_start_matches('r').parse().unwrap();
    assert!(n < 32, "register {s}");
    n
}

/// d in bits 8-4, r split over bit 9 and bits 3-0.
fn rd_rr(base: u16, d: u16, r: u16) -> u16 {
    base | ((r & 0x10) << 5) | (d << 4) | (r & 0xf)
}

/// Assembles one instruction into 16-bit words (little-endian program memory).
pub(super) fn asm_line(line: &str) -> Vec<u16> {
    let line = line.trim();
    let (mn, rest) = match line.split_once(' ') {
        Some((m, r)) => (m, r.trim()),
        None => (line, ""),
    };
    let ops: Vec<&str> = if rest.is_empty() { vec![] } else { rest.split(',').map(|s| s.trim()).collect() };
    let one = |w: u16| vec![w];
    match mn {
        "NOP" => one(0x0000),
        "RET" => one(0x9508),
        "RETI" => one(0x9518),
        "ICALL" => one(0x9509),
        "IJMP" => one(0x9409),
        "EICALL" => one(0x9519),
        "EIJMP" => one(0x9419),
        "WDR" => one(0x95a8),
        "LPM" | "ELPM" if ops.is_empty() => one(if mn == "LPM" { 0x95c8 } else { 0x95d8 }),
        "LPM" | "ELPM" => {
            let base = match (mn, ops[1]) {
                ("LPM", "Z") => 0x9004,
                ("LPM", "Z+") => 0x9005,
                ("ELPM", "Z") => 0x9006,
                ("ELPM", "Z+") => 0x9007,
                _ => panic!("{line}"),
            };
            one(base | (reg(ops[0]) << 4))
        }
        "ADD" => one(rd_rr(0x0c00, reg(ops[0]), reg(ops[1]))),
        "ADC" => one(rd_rr(0x1c00, reg(ops[0]), reg(ops[1]))),
        "SUB" => one(rd_rr(0x1800, reg(ops[0]), reg(ops[1]))),
        "SBC" => one(rd_rr(0x0800, reg(ops[0]), reg(ops[1]))),
        "CPC" => one(rd_rr(0x0400, reg(ops[0]), reg(ops[1]))),
        "CPSE" => one(rd_rr(0x1000, reg(ops[0]), reg(ops[1]))),
        "MOV" => one(rd_rr(0x2c00, reg(ops[0]), reg(ops[1]))),
        "MUL" => one(rd_rr(0x9c00, reg(ops[0]), reg(ops[1]))),
        "MOVW" => one(0x0100 | ((reg(ops[0]) / 2) << 4) | (reg(ops[1]) / 2)),
        "MULS" => one(0x0200 | ((reg(ops[0]) - 16) << 4) | (reg(ops[1]) - 16)),
        "MULSU" => one(0x0300 | ((reg(ops[0]) - 16) << 4) | (reg(ops[1]) - 16)),
        "LDI" | "CPI" | "SBCI" => {
            let base = match mn { "LDI" => 0xe000, "CPI" => 0x3000, _ => 0x4000 };
            let k = num(ops[1]) as u16 & 0xff;
            one(base | ((k & 0xf0) << 4) | ((reg(ops[0]) - 16) << 4) | (k & 0xf))
        }
        "BREAK" => one(0x9598),
        "CLR" => one(rd_rr(0x2400, reg(ops[0]), reg(ops[0]))),
        "EOR" => one(rd_rr(0x2400, reg(ops[0]), reg(ops[1]))),
        "ANDI" => {
            let k = num(ops[1]) as u16 & 0xff;
            one(0x7000 | ((k & 0xf0) << 4) | ((reg(ops[0]) - 16) << 4) | (k & 0xf))
        }
        "BRNE" => one(0xf400 | (((num(ops[0]) / 2) as u16 & 0x7f) << 3) | 1),
        "INC" => one(0x9403 | (reg(ops[0]) << 4)),
        "SWAP" => one(0x9402 | (reg(ops[0]) << 4)),
        "ROR" => one(0x9407 | (reg(ops[0]) << 4)),
        "LSR" => one(0x9406 | (reg(ops[0]) << 4)),
        "NEG" => one(0x9401 | (reg(ops[0]) << 4)),
        "PUSH" => one(0x920f | (reg(ops[0]) << 4)),
        "POP" => one(0x900f | (reg(ops[0]) << 4)),
        "BCLR" => one(0x9488 | ((num(ops[0]) as u16) << 4)),
        "BLD" => one(0xf800 | (reg(ops[0]) << 4) | num(ops[1]) as u16),
        "BRBS" | "BRBC" => {
            let k = (num(ops[1]) / 2) as u16 & 0x7f;
            one((if mn == "BRBS" { 0xf000 } else { 0xf400 }) | (k << 3) | num(ops[0]) as u16)
        }
        "RJMP" => one(0xc000 | ((num(ops[0]) / 2) as u16 & 0xfff)),
        "RCALL" => one(0xd000 | ((num(ops[0]) / 2) as u16 & 0xfff)),
        "JMP" | "CALL" => {
            let k = (num(ops[0]) / 2) as u32;
            let first = if mn == "JMP" { 0x940c } else { 0x940e } | ((k >> 16) & 1) as u16 | ((((k >> 17) & 0x1f) as u16) << 4);
            vec![first, (k & 0xffff) as u16]
        }
        "IN" => {
            let a = num(ops[1]) as u16;
            one(0xb000 | ((a & 0x30) << 5) | (reg(ops[0]) << 4) | (a & 0xf))
        }
        "OUT" => {
            let a = num(ops[0]) as u16;
            one(0xb800 | ((a & 0x30) << 5) | (reg(ops[1]) << 4) | (a & 0xf))
        }
        "SBI" | "CBI" | "SBIC" | "SBIS" => {
            let base = match mn { "CBI" => 0x9800, "SBIC" => 0x9900, "SBI" => 0x9a00, _ => 0x9b00 };
            one(base | ((num(ops[0]) as u16) << 3) | num(ops[1]) as u16)
        }
        "LDS" => vec![0x9000 | (reg(ops[0]) << 4), num(ops[1]) as u16],
        "STS" => vec![0x9200 | (reg(ops[1]) << 4), num(ops[0]) as u16],
        "LAC" | "LAS" | "LAT" | "XCH" => {
            assert_eq!(ops[0], "Z");
            let low = match mn { "LAC" => 6, "LAS" => 5, "LAT" => 7, _ => 4 };
            one(0x9200 | (reg(ops[1]) << 4) | low)
        }
        "LD" | "ST" => {
            let (r, p) = if mn == "LD" { (ops[0], ops[1]) } else { (ops[1], ops[0]) };
            let load = mn == "LD";
            let (ptr, kind) = match p {
                "X" => ('X', 0), "X+" => ('X', 1), "-X" => ('X', 2),
                "Y" => ('Y', 0), "Y+" => ('Y', 1), "-Y" => ('Y', 2),
                "Z" => ('Z', 0), "Z+" => ('Z', 1), "-Z" => ('Z', 2),
                _ => panic!("{line}"),
            };
            // LD/ST encodings: X ...1100/1101/1110; Y 1000 (plain, 1000 0..),
            // 1001, 1010; Z 0000 (plain), 0001, 0010.
            let (hi, low) = match (ptr, kind) {
                ('X', k) => (0x9000, 0xc + k),
                ('Y', 0) => (0x8000, 0x8),
                ('Y', k) => (0x9000, 0x8 + k),
                ('Z', 0) => (0x8000, 0x0),
                ('Z', k) => (0x9000, k),
                _ => unreachable!(),
            };
            one(hi | if load { 0 } else { 0x0200 } | (reg(r) << 4) | low as u16)
        }
        "LDD" | "STD" => {
            let (r, p) = if mn == "LDD" { (ops[0], ops[1]) } else { (ops[1], ops[0]) };
            let q = num(&p[2..]) as u16;
            let qbits = (q & 7) | ((q & 0x18) << 7) | ((q & 0x20) << 8);
            let y = p.starts_with('Y');
            one(0x8000 | if mn == "STD" { 0x0200 } else { 0 } | qbits | (reg(r) << 4) | if y { 0x8 } else { 0 })
        }
        _ => panic!("assembler: {line}"),
    }
}

pub(super) fn load(cpu: &mut Cpu, lines: &[&str]) {
    let mut at = 0;
    for l in lines {
        for w in asm_line(l) {
            cpu.prog[at] = w;
            at += 1;
        }
    }
}

fn set_prog_byte(cpu: &mut Cpu, i: usize, v: u8) {
    let w = cpu.prog[i >> 1];
    cpu.prog[i >> 1] = if i & 1 == 0 { (w & 0xff00) | v as u16 } else { (w & 0x00ff) | ((v as u16) << 8) };
}

// ---- cpu.spec.ts, by hand -------------------------------------------------------------

/// upstream: `should set initial value of SP to the last byte of internal SRAM`
#[test]
fn cpu_initial_sp_is_the_last_byte_of_sram() {
    let cpu = Cpu::new(vec![0; 1024], 0x1000);
    assert_eq!(cpu.sp(), 0x10ff);
}

fn run_events(order: &[i64], rounds: usize, adjust: impl Fn(&mut Cpu)) -> Vec<(i64, u64)> {
    let mut cpu = Cpu::new(vec![0; 1024], 0x1000);
    for &i in order {
        cpu.add_clock_event(i as u32, i);
    }
    adjust(&mut cpu);
    let mut events = Vec::new();
    for _ in 0..rounds {
        cpu.cycles += 1;
        let now = cpu.cycles;
        cpu.tick_with(|_, id| events.push((id as i64, now)));
    }
    events
}

/// upstream: `should execute queued events after the given number of cycles has passed`
#[test]
fn cpu_events_run_after_the_given_cycles() {
    assert_eq!(run_events(&[1, 4, 10], 10, |_| {}), vec![(1, 1), (4, 4), (10, 10)]);
}

/// upstream: `should correctly sort the events when added in reverse order`
#[test]
fn cpu_events_sort_when_added_in_reverse() {
    assert_eq!(run_events(&[10, 4, 1], 10, |_| {}), vec![(1, 1), (4, 4), (10, 10)]);
}

/// upstream: `updateClockEvent should update the number of cycles for the given clock event`
#[test]
fn cpu_update_clock_event() {
    let ev = run_events(&[1, 4, 10], 14, |cpu| {
        cpu.update_clock_event(4, 2);
        cpu.update_clock_event(1, 12);
    });
    assert_eq!(ev, vec![(4, 2), (10, 10), (1, 12)]);
}

/// upstream: `clearClockEvent should remove the given clock event`
#[test]
fn cpu_clear_clock_event() {
    let ev = run_events(&[1, 4, 10], 10, |cpu| {
        cpu.clear_clock_event(4);
    });
    assert_eq!(ev, vec![(1, 1), (10, 10)]);
}

/// upstream: `should return false if the provided clock event is not scheduled`
#[test]
fn cpu_clear_clock_event_reports_absence() {
    let mut cpu = Cpu::new(vec![0; 1024], 0x1000);
    cpu.add_clock_event(4, 4);
    cpu.add_clock_event(10, 10);
    cpu.add_clock_event(1, 1);
    assert!(cpu.clear_clock_event(4));
    assert!(cpu.clear_clock_event(10));
    assert!(!cpu.clear_clock_event(4));
    assert!(!cpu.clear_clock_event(10));
}

// ---- interrupt.spec.ts, by hand --------------------------------------------------------------

/// upstream: `should execute interrupt handler`
#[test]
fn interrupt_executes_the_handler() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu.pc = 0x520;
    cpu.data[94] = 0;
    cpu.data[93] = 0x80;
    cpu.data[95] = 0b10000001;
    avr_interrupt(&mut cpu, 5);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.pc, 5);
    assert_eq!(cpu.data[93], 0x7e);
    assert_eq!(cpu.data[0x80], 0x20);
    assert_eq!(cpu.data[0x7f], 0x5);
    assert_eq!(cpu.data[95], 0b00000001);
}

/// upstream: `should push a 3-byte return address when running in 22-bit PC mode (issue #58)`
#[test]
fn interrupt_pushes_three_bytes_on_a_22_bit_pc() {
    let mut cpu = Cpu::new(vec![0; 0x80000], 8192);
    assert!(cpu.pc22_bits);
    cpu.pc = 0x10520;
    cpu.data[94] = 0;
    cpu.data[93] = 0x80;
    cpu.data[95] = 0b10000001;
    avr_interrupt(&mut cpu, 5);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.pc, 5);
    assert_eq!(cpu.data[93], 0x7d);
    assert_eq!(cpu.data[0x80], 0x20);
    assert_eq!(cpu.data[0x7f], 0x05);
    assert_eq!(cpu.data[0x7e], 0x1);
    assert_eq!(cpu.data[95], 0b00000001);
}

// ---- instruction.spec.ts ------------------------------------------------------------------------

/// upstream: `should execute 'ADC r0, r1' instruction when carry is on`
#[test]
fn execute_adc_r0_r1_instruction_when_carry_is_on() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ADC r0, r1"]);
    cpu.data[r0] = 10;
    cpu.data[r1] = 20;
    cpu.data[SREG] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 31);
    assert_eq!(cpu.data[SREG], 0);
}

/// upstream: `should execute 'ADC r0, r1' instruction when carry is on and the result overflows`
#[test]
fn execute_adc_r0_r1_instruction_when_carry_is_on_and_the_result_overflows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ADC r0, r1"]);
    cpu.data[r0] = 10;
    cpu.data[r1] = 245;
    cpu.data[SREG] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 0);
    assert_eq!(cpu.data[SREG], SREG_H | SREG_Z | SREG_C);
}

/// upstream: `should execute 'ADD r0, r1' instruction when result overflows`
#[test]
fn execute_add_r0_r1_instruction_when_result_overflows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ADD r0, r1"]);
    cpu.data[r0] = 11;
    cpu.data[r1] = 245;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 0);
    assert_eq!(cpu.data[SREG], SREG_H | SREG_Z | SREG_C);
}

/// upstream: `should execute 'ADD r0, r1' instruction when carry is on`
#[test]
fn execute_add_r0_r1_instruction_when_carry_is_on() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ADD r0, r1"]);
    cpu.data[r0] = 11;
    cpu.data[r1] = 244;
    cpu.data[SREG] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 255);
    assert_eq!(cpu.data[SREG], SREG_S | SREG_N);
}

/// upstream: `should execute 'ADD r0, r1' instruction when carry is on and the result overflows`
#[test]
fn execute_add_r0_r1_instruction_when_carry_is_on_and_the_result_overflows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ADD r0, r1"]);
    cpu.data[r0] = 11;
    cpu.data[r1] = 245;
    cpu.data[SREG] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 0);
    assert_eq!(cpu.data[SREG], SREG_H | SREG_Z | SREG_C);
}

/// upstream: `should execute 'BCLR 2' instruction`
#[test]
fn execute_bclr_2_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BCLR 2"]);
    cpu.data[SREG] = 0xff;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], 0xfb);
}

/// upstream: `should execute 'BLD r4, 7' instruction`
#[test]
fn execute_bld_r4_7_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BLD r4, 7"]);
    cpu.data[r4] = 0x15;
    cpu.data[SREG] = 0x40;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r4], 0x95);
    assert_eq!(cpu.data[SREG], 0x40);
}

/// upstream: `should execute 'BRBC 0, +8' instruction when SREG.C is clear`
#[test]
fn execute_brbc_0_8_instruction_when_sreg_c_is_clear() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BRBC 0, +8"]);
    cpu.data[SREG] = SREG_V;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1 + 8 / 2);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'BRBC 0, +8' instruction when SREG.C is set`
#[test]
fn execute_brbc_0_8_instruction_when_sreg_c_is_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BRBC 0, +8"]);
    cpu.data[SREG] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
}

/// upstream: `should execute 'BRBS 3, 92' instruction when SREG.V is set`
#[test]
fn execute_brbs_3_92_instruction_when_sreg_v_is_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BRBS 3, 92"]);
    cpu.data[SREG] = SREG_V;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1 + 92 / 2);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'BRBS 3, -4' instruction when SREG.V is set`
#[test]
fn execute_brbs_3_4_instruction_when_sreg_v_is_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BRBS 3, -4"]);
    cpu.data[SREG] = SREG_V;
    avr_instruction(&mut cpu);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0);
    assert_eq!(cpu.cycles, 3);  // 1 for NOP, 2 for BRBS
}

/// upstream: `should execute 'BRBS 3, -4' instruction when SREG.V is clear`
#[test]
fn execute_brbs_3_4_instruction_when_sreg_v_is_clear() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["BRBS 3, -4"]);
    cpu.data[SREG] = 0x0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
}

/// upstream: `should execute 'CALL' instruction`
#[test]
fn execute_call_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CALL 0xb8"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 150;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x5c);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.data[150], 2);  // return addr
    assert_eq!(cpu.data[SP], 148);  // SP should be decremented
}

/// upstream: `should push 3-byte return address when executing 'CALL' instruction on device with >128k flash`
#[test]
fn push_3_byte_return_address_when_executing_call_instruction_on_device_with_128k_flash() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["CALL 0xb8"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 150;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x5c);
    assert_eq!(cpu.cycles, 5);
    assert_eq!(cpu.data[150], 2);  // return addr
    assert_eq!(cpu.data[SP], 147);  // SP should be decremented by 3
}

/// upstream: `should execute 'CBI 0x0c, 5'`
#[test]
fn execute_cbi_0x0c_5() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CBI 0x0c, 5"]);
    cpu.data[0x2c] = 0b11111111;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[0x2c], 0b11011111);
}

/// upstream: `should execute 'CPC r27, r18' instruction`
#[test]
fn execute_cpc_r27_r18_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPC r27, r18"]);
    cpu.data[r18] = 0x1;
    cpu.data[r27] = 0x1;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], 0);
}

/// upstream: `should execute 'CPC r24, r1' instruction and set`
#[test]
fn execute_cpc_r24_r1_instruction_and_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPC r24, r1"]);
    cpu.data[r1] = 0;
    cpu.data[r24] = 0;
    cpu.data[SREG] = SREG_I | SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], SREG_I | SREG_H | SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'CPI r26, 0x9' instruction`
#[test]
fn execute_cpi_r26_0x9_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPI r26, 0x9"]);
    cpu.data[r26] = 0x8;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], SREG_H | SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'CPSE r2, r3' when r2 != r3`
#[test]
fn execute_cpse_r2_r3_when_r2_r3() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPSE r2, r3"]);
    cpu.data[r2] = 10;
    cpu.data[r3] = 11;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
}

/// upstream: `should execute 'CPSE r2, r3' when r2 == r3`
#[test]
fn execute_cpse_r2_r3_when_r2_r3_2() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPSE r2, r3"]);
    cpu.data[r2] = 10;
    cpu.data[r3] = 10;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 2);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'CPSE r2, r3' when r2 == r3 and followed by 2-word instruction`
#[test]
fn execute_cpse_r2_r3_when_r2_r3_and_followed_by_2_word_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["CPSE r2, r3", "CALL 8"]);
    cpu.data[r2] = 10;
    cpu.data[r3] = 10;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 3);
    assert_eq!(cpu.cycles, 3);
}

/// upstream: `should execute 'EICALL' instruction`
#[test]
fn execute_eicall_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["EICALL"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    cpu.data[EIND] = 1;
    cpu.set16(Z, 0x1234);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x11234);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.data[SP], 0x80 - 3);  // according to datasheet: SP ← SP - 3
    assert_eq!(cpu.data[0x80], 1);  // Return address
}

/// upstream: `should execute 'EIJMP' instruction`
#[test]
fn execute_eijmp_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["EIJMP"]);
    cpu.data[EIND] = 1;
    cpu.set16(Z, 0x1040);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x11040);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'ELPM' instruction`
#[test]
fn execute_elpm_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["ELPM"]);
    cpu.data[Z] = 0x50;
    cpu.data[RAMPZ] = 0x2;
    set_prog_byte(&mut cpu, 0x20050, 0x62);  // value to be loaded
    avr_instruction(&mut cpu);
    assert_eq!(cpu.data[r0], 0x62);  // check that value was loaded to r0
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
}

/// upstream: `should execute 'ELPM r5, Z' instruction`
#[test]
fn execute_elpm_r5_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["ELPM r5, Z"]);
    cpu.data[Z] = 0x11;
    cpu.data[RAMPZ] = 0x1;
    set_prog_byte(&mut cpu, 0x10011, 0x99);  // value to be loaded
    avr_instruction(&mut cpu);
    assert_eq!(cpu.data[r5], 0x99);  // check that value was loaded to r5
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
}

/// upstream: `should execute 'ELPM r6, Z+' instruction`
#[test]
fn execute_elpm_r6_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["ELPM r6, Z+"]);
    cpu.set16(Z, 0xffff);
    cpu.data[RAMPZ] = 0x2;
    set_prog_byte(&mut cpu, 0x2ffff, 0x22);  // value to be loaded
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.data[r6], 0x22);  // check that value was loaded to r6
    assert_eq!(cpu.get16(Z), 0x0);  // verify that Z was incremented
    assert_eq!(cpu.data[RAMPZ], 3);  // verify that RAMPZ was incremented
}

/// upstream: `should clamp RAMPZ when executing 'ELPM r6, Z+' instruction`
#[test]
fn clamp_rampz_when_executing_elpm_r6_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["ELPM r6, Z+"]);
    cpu.set16(Z, 0xffff);
    cpu.data[RAMPZ] = 0x3;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.data[RAMPZ], 0x0);  // verify that RAMPZ was reset to zero
}

/// upstream: `should execute 'ICALL' instruction`
#[test]
fn execute_icall_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ICALL"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    cpu.set16(Z, 0x2020);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.pc, 0x2020);
    assert_eq!(cpu.data[0x80], 1);  // Return address
    assert_eq!(cpu.data[SP], 0x7e);  // SP Should decrement by 2
}

/// upstream: `should push 3-byte return address when executing 'ICALL' instruction on device with >128k flash`
#[test]
fn push_3_byte_return_address_when_executing_icall_instruction_on_device_with_128k_flash() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["ICALL"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    cpu.set16(Z, 0x2020);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.pc, 0x2020);
    assert_eq!(cpu.data[0x80], 1);  // Return address
    assert_eq!(cpu.data[SP], 0x7d);  // SP Should decrement by 3
}

/// upstream: `should execute 'IJMP' instruction`
#[test]
fn execute_ijmp_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["IJMP"]);
    cpu.set16(Z, 0x1040);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.pc, 0x1040);
}

/// upstream: `should execute 'IN r5, 0xb' instruction`
#[test]
fn execute_in_r5_0xb_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["IN r5, 0xb"]);
    cpu.data[0x2b] = 0xaf;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.data[r5], 0xaf);
}

/// upstream: `should execute 'INC r5' instruction`
#[test]
fn execute_inc_r5_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["INC r5"]);
    cpu.data[r5] = 0x7f;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.data[r5], 0x80);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], SREG_N | SREG_V);
}

/// upstream: `should execute 'INC r5' instruction when r5 == 0xff`
#[test]
fn execute_inc_r5_instruction_when_r5_0xff() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["INC r5"]);
    cpu.data[r5] = 0xff;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.data[r5], 0);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], SREG_Z);
}

/// upstream: `should execute 'JMP 0xb8' instruction`
#[test]
fn execute_jmp_0xb8_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["JMP 0xb8"]);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x5c);
    assert_eq!(cpu.cycles, 3);
}

/// upstream: `should execute 'LAC Z, r19' instruction`
#[test]
fn execute_lac_z_r19_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LAC Z, r19"]);
    cpu.data[r19] = 0x02;
    cpu.set16(Z, 0x100);
    cpu.data[0x100] = 0x96;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r19], 0x96);
    assert_eq!(cpu.get16(Z), 0x100);
    assert_eq!(cpu.data[0x100], 0x94);
}

/// upstream: `should execute 'LAS Z, r17' instruction`
#[test]
fn execute_las_z_r17_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LAS Z, r17"]);
    cpu.data[r17] = 0x11;
    cpu.data[Z] = 0x80;
    cpu.data[0x80] = 0x44;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r17], 0x44);
    assert_eq!(cpu.data[Z], 0x80);
    assert_eq!(cpu.data[0x80], 0x55);
}

/// upstream: `should execute 'LAT Z, r0' instruction`
#[test]
fn execute_lat_z_r0_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LAT Z, r0"]);
    cpu.data[r0] = 0x33;
    cpu.data[Z] = 0x80;
    cpu.data[0x80] = 0x66;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 0x66);
    assert_eq!(cpu.data[Z], 0x80);
    assert_eq!(cpu.data[0x80], 0x55);
}

/// upstream: `should execute 'LD r1, X' instruction`
#[test]
fn execute_ld_r1_x_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r1, X"]);
    cpu.data[0xc0] = 0x15;
    cpu.data[X] = 0xc0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r1], 0x15);
    assert_eq!(cpu.data[X], 0xc0);  // verify that X was unchanged
}

/// upstream: `should execute 'LD r17, X+' instruction`
#[test]
fn execute_ld_r17_x_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r17, X+"]);
    cpu.data[0xc0] = 0x15;
    cpu.data[X] = 0xc0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r17], 0x15);
    assert_eq!(cpu.data[X], 0xc1);  // verify that X was incremented
}

/// upstream: `should execute 'LD r1, -X' instruction`
#[test]
fn execute_ld_r1_x_instruction_2() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r1, -X"]);
    cpu.data[0x98] = 0x22;
    cpu.data[X] = 0x99;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r1], 0x22);
    assert_eq!(cpu.data[X], 0x98);  // verify that X was decremented
}

/// upstream: `should execute 'LD r8, Y' instruction`
#[test]
fn execute_ld_r8_y_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r8, Y"]);
    cpu.data[0xc0] = 0x15;
    cpu.data[Y] = 0xc0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[8], 0x15);
    assert_eq!(cpu.data[Y], 0xc0);  // verify that Y was unchanged
}

/// upstream: `should execute 'LD r3, Y+' instruction`
#[test]
fn execute_ld_r3_y_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r3, Y+"]);
    cpu.data[0xc0] = 0x15;
    cpu.data[Y] = 0xc0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r3], 0x15);
    assert_eq!(cpu.data[Y], 0xc1);  // verify that Y was incremented
}

/// upstream: `should execute 'LD r0, -Y' instruction`
#[test]
fn execute_ld_r0_y_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r0, -Y"]);
    cpu.data[0x98] = 0x22;
    cpu.data[Y] = 0x99;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r0], 0x22);
    assert_eq!(cpu.data[Y], 0x98);  // verify that Y was decremented
}

/// upstream: `should execute 'LDD r4, Y+2' instruction`
#[test]
fn execute_ldd_r4_y_2_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LDD r4, Y+2"]);
    cpu.data[0x82] = 0x33;
    cpu.data[Y] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r4], 0x33);
    assert_eq!(cpu.data[Y], 0x80);  // verify that Y was unchanged
}

/// upstream: `should execute 'LD r5, Z' instruction`
#[test]
fn execute_ld_r5_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r5, Z"]);
    cpu.data[0xcc] = 0xf5;
    cpu.data[Z] = 0xcc;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r5], 0xf5);
    assert_eq!(cpu.data[Z], 0xcc);  // verify that Z was unchanged
}

/// upstream: `should execute 'LD r7, Z+' instruction`
#[test]
fn execute_ld_r7_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r7, Z+"]);
    cpu.data[0xc0] = 0x25;
    cpu.data[Z] = 0xc0;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r7], 0x25);
    assert_eq!(cpu.data[Z], 0xc1);  // verify that Y was incremented
}

/// upstream: `should execute 'LD r0, -Z' instruction`
#[test]
fn execute_ld_r0_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LD r0, -Z"]);
    cpu.data[0x9e] = 0x66;
    cpu.data[Z] = 0x9f;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r0], 0x66);
    assert_eq!(cpu.data[Z], 0x9e);  // verify that Y was decremented
}

/// upstream: `should execute 'LDD r15, Z+31' instruction`
#[test]
fn execute_ldd_r15_z_31_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LDD r15, Z+31"]);
    cpu.data[0x9f] = 0x33;
    cpu.data[Z] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[15], 0x33);
    assert_eq!(cpu.data[Z], 0x80);  // verify that Z was unchanged
}

/// upstream: `should execute 'LDI r28, 0xff' instruction`
#[test]
fn execute_ldi_r28_0xff_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LDI r28, 0xff"]);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[Y], 0xff);
}

/// upstream: `should execute 'LDS r5, 0x150' instruction`
#[test]
fn execute_lds_r5_0x150_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LDS r5, 0x150"]);
    cpu.data[0x150] = 0x7a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x2);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[r5], 0x7a);
}

/// upstream: `should execute 'LPM' instruction`
#[test]
fn execute_lpm_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LPM"]);
    cpu.prog[0x40] = 0xa0;
    cpu.data[Z] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.data[r0], 0xa0);
    assert_eq!(cpu.data[Z], 0x80);  // verify that Z was unchanged
}

/// upstream: `should execute 'LPM r2, Z' instruction`
#[test]
fn execute_lpm_r2_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LPM r2, Z"]);
    cpu.prog[0x40] = 0xa0;
    cpu.data[Z] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.data[r2], 0xa0);
    assert_eq!(cpu.data[Z], 0x80);  // verify that Z was unchanged
}

/// upstream: `should execute 'LPM r1, Z+' instruction`
#[test]
fn execute_lpm_r1_z_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LPM r1, Z+"]);
    cpu.prog[0x40] = 0xa0;
    cpu.data[Z] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.data[r1], 0xa0);
    assert_eq!(cpu.data[Z], 0x81);  // verify that Z was incremented
}

/// upstream: `should execute 'LSR r7' instruction`
#[test]
fn execute_lsr_r7_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["LSR r7"]);
    cpu.data[r7] = 0x45;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r7], 0x22);
    assert_eq!(cpu.data[SREG], SREG_S | SREG_V | SREG_C);
}

/// upstream: `should execute 'MOV r7, r8' instruction`
#[test]
fn execute_mov_r7_r8_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MOV r7, r8"]);
    cpu.data[r8] = 0x45;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r7], 0x45);
}

/// upstream: `should execute 'MOVW r26, r22' instruction`
#[test]
fn execute_movw_r26_r22_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MOVW r26, r22"]);
    cpu.data[r22] = 0x45;
    cpu.data[r23] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[X], 0x45);
    assert_eq!(cpu.data[r27], 0x9a);
}

/// upstream: `should execute 'MUL r5, r6' instruction`
#[test]
fn execute_mul_r5_r6_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MUL r5, r6"]);
    cpu.data[r5] = 100;
    cpu.data[r6] = 5;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.get16(0), 500);
    assert_eq!(cpu.data[SREG], 0);
}

/// upstream: `should execute 'MUL r5, r6' instruction and update carry flag when numbers are big`
#[test]
fn execute_mul_r5_r6_instruction_and_update_carry_flag_when_numbers_are_big() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MUL r5, r6"]);
    cpu.data[r5] = 200;
    cpu.data[r6] = 200;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.get16(0), 40000);
    assert_eq!(cpu.data[SREG], SREG_C);
}

/// upstream: `should execute 'MUL r0, r1' and update the zero flag`
#[test]
fn execute_mul_r0_r1_and_update_the_zero_flag() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MUL r0, r1"]);
    cpu.data[r0] = 0;
    cpu.data[r1] = 9;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.get16(0), 0);
    assert_eq!(cpu.data[SREG], SREG_Z);
}

/// upstream: `should execute 'MULS r18, r19' instruction`
#[test]
fn execute_muls_r18_r19_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MULS r18, r19"]);
    cpu.data[r18] = (-5i8) as u8; // a negative number stored in a Uint8Array
    cpu.data[r19] = 100;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!((cpu.get16(0) as i16), -500);
    assert_eq!(cpu.data[SREG], SREG_C);
}

/// upstream: `should execute 'MULSU r16, r17' instruction`
#[test]
fn execute_mulsu_r16_r17_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["MULSU r16, r17"]);
    cpu.data[r16] = (-5i8) as u8; // a negative number stored in a Uint8Array
    cpu.data[r17] = 200;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!((cpu.get16(0) as i16), -1000);
    assert_eq!(cpu.data[SREG], SREG_C);
}

/// upstream: `should execute 'NEG r20' instruction`
#[test]
fn execute_neg_r20_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["NEG r20"]);
    cpu.data[r20] = 0x56;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[20], 0xaa);
    assert_eq!(cpu.data[SREG], SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'NOP' instruction`
#[test]
fn execute_nop_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["NOP"]);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
}

/// upstream: `should execute 'OUT 0x3f, r1' instruction`
#[test]
fn execute_out_0x3f_r1_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["OUT 0x3f, r1"]);
    cpu.data[r1] = 0x5a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[0x5f], 0x5a);
}

/// upstream: `should execute 'POP r26' instruction`
#[test]
fn execute_pop_r26_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["POP r26"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0xff;
    cpu.data[0x100] = 0x1a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[X], 0x1a);
    assert_eq!(cpu.get16(SP), 0x100);
}

/// upstream: `should execute 'PUSH r11' instruction`
#[test]
fn execute_push_r11_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["PUSH r11"]);
    cpu.data[11] = 0x2a;
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0xff;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0xff], 0x2a);
    assert_eq!(cpu.get16(SP), 0xfe);
}

/// upstream: `should execute 'RCALL .+6' instruction`
#[test]
fn execute_rcall_6_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["RCALL 6"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 4);
    assert_eq!(cpu.cycles, 3);
    assert_eq!(cpu.get16(0x80), 1);  // RET address
    assert_eq!(cpu.data[SP], 0x7e);  // SP should decrement by 2
}

/// upstream: `should execute 'RCALL .-4' instruction`
#[test]
fn execute_rcall_4_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["NOP", "RCALL -4"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    avr_instruction(&mut cpu);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0);
    assert_eq!(cpu.cycles, 4);  // 1 for NOP, 3 for RCALL
    assert_eq!(cpu.get16(0x80), 2);  // RET address
    assert_eq!(cpu.data[SP], 0x7e);  // SP should decrement by 2
}

/// upstream: `should push 3-byte return address when executing 'RCALL' instruction on device with >128k flash`
#[test]
fn push_3_byte_return_address_when_executing_rcall_instruction_on_device_with_128k_flash() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["RCALL 6"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x80;
    cpu.set16(Z, 0x2020);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 4);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.get16(0x80), 1);  // RET address
    assert_eq!(cpu.data[SP], 0x7d);  // SP Should decrement by 3
}

/// upstream: `should execute 'RET' instruction`
#[test]
fn execute_ret_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["RET"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x90;
    cpu.data[0x92] = 16;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 16);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.data[SP], 0x92);  // SP should increment by 2
}

/// upstream: `should execute 'RET' instruction on device with >128k flash`
#[test]
fn execute_ret_instruction_on_device_with_128k_flash() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["RET"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0x90;
    cpu.data[0x91] = 0x1;
    cpu.data[0x93] = 0x16;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x10016);
    assert_eq!(cpu.cycles, 5);
    assert_eq!(cpu.data[SP], 0x93);  // SP should increment by 3
}

/// upstream: `should execute 'RETI' instruction`
#[test]
fn execute_reti_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["RETI"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0xc0;
    cpu.data[0xc2] = 200;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 200);
    assert_eq!(cpu.cycles, 4);
    assert_eq!(cpu.data[SP], 0xc2);  // SP should increment by 2
    assert_eq!(cpu.data[SREG], SREG_I);
}

/// upstream: `should execute 'RETI' instruction on device with >128k flash`
#[test]
fn execute_reti_instruction_on_device_with_128k_flash() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    cpu = Cpu::new(vec![0; 0x20000], 8192);
    load(&mut cpu, &["RETI"]);
    cpu.data[SPH] = 0;
    cpu.data[SP] = 0xc0;
    cpu.data[0xc1] = 0x1;
    cpu.data[0xc3] = 0x30;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 0x10030);
    assert_eq!(cpu.cycles, 5);
    assert_eq!(cpu.data[SP], 0xc3);  // SP should increment by 3
    assert_eq!(cpu.data[SREG], SREG_I);
}

/// upstream: `should execute 'RJMP 2' instruction`
#[test]
fn execute_rjmp_2_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["RJMP 2"]);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 2);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'ROR r0' instruction`
#[test]
fn execute_ror_r0_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ROR r0"]);
    cpu.data[r0] = 0x11;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 0x08);  // r0 should be right-shifted
    assert_eq!(cpu.data[SREG], SREG_S | SREG_V | SREG_C);
}

/// upstream: `should execute 'SBC r0, r1' instruction when carry is on and result overflows`
#[test]
fn execute_sbc_r0_r1_instruction_when_carry_is_on_and_result_overflows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBC r0, r1"]);
    cpu.data[r0] = 0;
    cpu.data[r1] = 10;
    cpu.data[95] = SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 245);
    assert_eq!(cpu.data[SREG], SREG_H | SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'SBCI r23, 3'`
#[test]
fn execute_sbci_r23_3() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBCI r23, 3"]);
    cpu.data[r23] = 3;
    cpu.data[SREG] = SREG_I | SREG_C;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[SREG], SREG_I | SREG_H | SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'SBI 0x0c, 5'`
#[test]
fn execute_sbi_0x0c_5() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBI 0x0c, 5"]);
    cpu.data[0x2c] = 0b00001111;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x2c], 0b00101111);
}

/// upstream: `should execute 'SBIS 0x0c, 5' when bit is clear`
#[test]
fn execute_sbis_0x0c_5_when_bit_is_clear() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBIS 0x0c, 5"]);
    cpu.data[0x2c] = 0b00001111;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
}

/// upstream: `should execute 'SBIS 0x0c, 5' when bit is set`
#[test]
fn execute_sbis_0x0c_5_when_bit_is_set() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBIS 0x0c, 5"]);
    cpu.data[0x2c] = 0b00101111;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 2);
    assert_eq!(cpu.cycles, 2);
}

/// upstream: `should execute 'SBIS 0x0c, 5' when bit is set and followed by 2-word instruction`
#[test]
fn execute_sbis_0x0c_5_when_bit_is_set_and_followed_by_2_word_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SBIS 0x0c, 5", "CALL 0xb8"]);
    cpu.data[0x2c] = 0b00101111;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 3);
    assert_eq!(cpu.cycles, 3);
}

/// upstream: `should execute 'ST X, r1' instruction`
#[test]
fn execute_st_x_r1_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST X, r1"]);
    cpu.data[r1] = 0x5a;
    cpu.data[X] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x9a], 0x5a);
    assert_eq!(cpu.data[X], 0x9a);  // verify that X was unchanged
}

/// upstream: `should execute 'ST X+, r1' instruction`
#[test]
fn execute_st_x_r1_instruction_2() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST X+, r1"]);
    cpu.data[r1] = 0x5a;
    cpu.data[X] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x9a], 0x5a);
    assert_eq!(cpu.data[X], 0x9b);  // verify that X was incremented
}

/// upstream: `should execute 'ST -X, r17' instruction`
#[test]
fn execute_st_x_r17_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST -X, r17"]);
    cpu.data[r17] = 0x88;
    cpu.data[X] = 0x99;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x98], 0x88);
    assert_eq!(cpu.data[X], 0x98);  // verify that X was decremented
}

/// upstream: `should execute 'ST Y, r2' instruction`
#[test]
fn execute_st_y_r2_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST Y, r2"]);
    cpu.data[r2] = 0x5b;
    cpu.data[Y] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x9a], 0x5b);
    assert_eq!(cpu.data[Y], 0x9a);  // verify that Y was unchanged
}

/// upstream: `should execute 'ST Y+, r1' instruction`
#[test]
fn execute_st_y_r1_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST Y+, r1"]);
    cpu.data[r1] = 0x5a;
    cpu.data[Y] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x9a], 0x5a);
    assert_eq!(cpu.data[Y], 0x9b);  // verify that Y was incremented
}

/// upstream: `should execute 'ST -Y, r1' instruction`
#[test]
fn execute_st_y_r1_instruction_2() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST -Y, r1"]);
    cpu.data[r1] = 0x5a;
    cpu.data[Y] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x99], 0x5a);
    assert_eq!(cpu.data[Y], 0x99);  // verify that Y was decremented
}

/// upstream: `should execute 'STD Y+17, r0' instruction`
#[test]
fn execute_std_y_17_r0_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["STD Y+17, r0"]);
    cpu.data[r0] = 0xba;
    cpu.data[Y] = 0x9a;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x9a + 17], 0xba);
    assert_eq!(cpu.data[Y], 0x9a);  // verify that Y was unchanged
}

/// upstream: `should execute 'ST Z, r16' instruction`
#[test]
fn execute_st_z_r16_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST Z, r16"]);
    cpu.data[r16] = 0xdf;
    cpu.data[Z] = 0x40;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x40], 0xdf);
    assert_eq!(cpu.data[Z], 0x40);  // verify that Z was unchanged
}

/// upstream: `should execute 'ST Z+, r0' instruction`
#[test]
fn execute_st_z_r0_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST Z+, r0"]);
    cpu.data[r0] = 0x55;
    cpu.set16(Z, 0x155);
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x155], 0x55);
    assert_eq!(cpu.get16(Z), 0x156);  // verify that Z was incremented
}

/// upstream: `should execute 'ST -Z, r16' instruction`
#[test]
fn execute_st_z_r16_instruction_2() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["ST -Z, r16"]);
    cpu.data[r16] = 0x5a;
    cpu.data[Z] = 0xff;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0xfe], 0x5a);
    assert_eq!(cpu.data[Z], 0xfe);  // verify that Z was decremented
}

/// upstream: `should execute 'STD Z+1, r0' instruction`
#[test]
fn execute_std_z_1_r0_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["STD Z+1, r0"]);
    cpu.data[r0] = 0xcc;
    cpu.data[Z] = 0x50;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x51], 0xcc);
    assert_eq!(cpu.data[Z], 0x50);  // verify that Z was unchanged
}

/// upstream: `should execute 'STS 0x151, r31' instruction`
#[test]
fn execute_sts_0x151_r31_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["STS 0x151, r31"]);
    cpu.data[r31] = 0x80;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 2);
    assert_eq!(cpu.cycles, 2);
    assert_eq!(cpu.data[0x151], 0x80);
}

/// upstream: `should execute 'SUB r0, r1' instruction when result overflows`
#[test]
fn execute_sub_r0_r1_instruction_when_result_overflows() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SUB r0, r1"]);
    cpu.data[r0] = 0;
    cpu.data[r1] = 10;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r0], 246);
    assert_eq!(cpu.data[SREG], SREG_S | SREG_N | SREG_C);
}

/// upstream: `should execute 'SWAP r1' instruction`
#[test]
fn execute_swap_r1_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["SWAP r1"]);
    cpu.data[r1] = 0xa5;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r1], 0x5a);
}

/// upstream: `should execute 'WDR' instruction and call 'cpu.onWatchdogReset'`
#[test]
fn execute_wdr_instruction_and_call_cpu_onwatchdogreset() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["WDR"]);
    // the port counts WDR instructions in cpu.watchdog_resets
    assert_eq!(cpu.watchdog_resets, 0);
    avr_instruction(&mut cpu);
    assert!(cpu.watchdog_resets > 0);
}

/// upstream: `should execute 'XCH Z, r21' instruction`
#[test]
fn execute_xch_z_r21_instruction() {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    load(&mut cpu, &["XCH Z, r21"]);
    cpu.data[r21] = 0xa1;
    cpu.data[Z] = 0x50;
    cpu.data[0x50] = 0xb9;
    avr_instruction(&mut cpu);
    assert_eq!(cpu.pc, 1);
    assert_eq!(cpu.cycles, 1);
    assert_eq!(cpu.data[r21], 0xb9);
    assert_eq!(cpu.data[0x50], 0xa1);
}
