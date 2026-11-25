//! The Cortex-M0+ core, ported from rp2040js 1.3.3 (MIT):
//! `src/cortex-m0-core.ts`.
//!
//! The decoder is the reference's if-chain in the reference's order, so an
//! encoding two arms could match resolves the same way. Arithmetic that the
//! reference does on JavaScript doubles (sums that pass 2^32, shift counts
//! JavaScript masks to five bits) is done here in i64 or with the same
//! masking, so flags and results agree bit for bit, including where the
//! reference departs from ARMv6-M (each such place is marked "reference
//! quirk" and has a unit test).

use super::{Rp2040, Stop};

pub const EXC_RESET: u32 = 1;
pub const EXC_NMI: u32 = 2;
pub const EXC_HARDFAULT: u32 = 3;
pub const EXC_SVCALL: u32 = 11;
pub const EXC_PENDSV: u32 = 14;
pub const EXC_SYSTICK: u32 = 15;

pub const SYSM_APSR: u32 = 0;
pub const SYSM_XPSR: u32 = 3;
pub const SYSM_IPSR: u32 = 5;
pub const SYSM_MSP: u32 = 8;
pub const SYSM_PSP: u32 = 9;
pub const SYSM_PRIMASK: u32 = 16;
pub const SYSM_CONTROL: u32 = 20;

const LOWEST_PRIORITY: i32 = 4;
/// `IRQ.RTC`: interrupts above it are software-only and clear on entry.
pub const MAX_HARDWARE_IRQ: u32 = 25;

const SP: usize = 13;
const LR: usize = 14;
const PC: usize = 15;

pub const SIO_START: u32 = 0xd000_0000;
pub const APB_START: u32 = 0x4000_0000;

#[derive(Clone)]
pub struct Cpu {
    pub r: [u32; 16],
    pub banked_sp: u32,
    pub cycles: u64,
    pub event_registered: bool,
    pub waiting: bool,
    pub n: bool,
    pub z: bool,
    pub c: bool,
    pub v: bool,
    pub break_rewind: u32,
    pub pm: bool,
    /// CONTROL.SPSEL: true when the process stack is selected.
    pub spsel: bool,
    pub npriv: bool,
    /// true in Handler mode.
    pub handler_mode: bool,
    pub ipsr: u32,
    pub pending_interrupts: u32,
    pub enabled_interrupts: u32,
    pub interrupt_priorities: [u32; 4],
    pub pending_nmi: bool,
    pub pending_pendsv: bool,
    pub pending_svcall: bool,
    pub pending_systick: bool,
    pub interrupts_updated: bool,
    pub vtor: u32,
    pub shpr2: u32,
    pub shpr3: u32,
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Cpu {
    pub fn new() -> Self {
        let mut r = [0u32; 16];
        r[SP] = 0xffff_fffc;
        Cpu {
            r,
            banked_sp: 0xffff_fffc,
            cycles: 0,
            event_registered: false,
            waiting: false,
            n: false,
            z: false,
            c: false,
            v: false,
            break_rewind: 0,
            pm: false,
            spsel: false,
            npriv: false,
            handler_mode: false,
            ipsr: 0,
            pending_interrupts: 0,
            enabled_interrupts: 0,
            interrupt_priorities: [0xffff_ffff, 0, 0, 0],
            pending_nmi: false,
            pending_pendsv: false,
            pending_svcall: false,
            pending_systick: false,
            interrupts_updated: false,
            vtor: 0,
            shpr2: 0,
            shpr3: 0,
        }
    }

    pub fn sp(&self) -> u32 {
        self.r[SP]
    }
    pub fn set_sp(&mut self, v: u32) {
        self.r[SP] = v & !3;
    }
    pub fn pc(&self) -> u32 {
        self.r[PC]
    }

    pub fn apsr(&self) -> u32 {
        (if self.n { 0x8000_0000 } else { 0 })
            | (if self.z { 0x4000_0000 } else { 0 })
            | (if self.c { 0x2000_0000 } else { 0 })
            | (if self.v { 0x1000_0000 } else { 0 })
    }
    pub fn set_apsr(&mut self, v: u32) {
        self.n = v & 0x8000_0000 != 0;
        self.z = v & 0x4000_0000 != 0;
        self.c = v & 0x2000_0000 != 0;
        self.v = v & 0x1000_0000 != 0;
    }
    pub fn xpsr(&self) -> u32 {
        self.apsr() | self.ipsr | (1 << 24)
    }
    pub fn set_xpsr(&mut self, v: u32) {
        self.set_apsr(v);
        self.ipsr = v & 0x3f;
    }

    pub fn check_condition(&self, cond: u32) -> bool {
        let result = match cond >> 1 {
            0b000 => self.z,
            0b001 => self.c,
            0b010 => self.n,
            0b011 => self.v,
            0b100 => self.c && !self.z,
            0b101 => self.n == self.v,
            0b110 => self.n == self.v && !self.z,
            _ => true,
        };
        if cond & 1 != 0 && cond != 0b1111 {
            !result
        } else {
            result
        }
    }

    /// `switchStack`.
    pub fn switch_stack(&mut self, process: bool) {
        if self.spsel != process {
            let temp = self.sp();
            self.set_sp(self.banked_sp);
            self.banked_sp = temp;
            self.spsel = process;
        }
    }
    pub fn sp_process(&self) -> u32 {
        if self.spsel {
            self.sp()
        } else {
            self.banked_sp
        }
    }
    pub fn set_sp_process(&mut self, v: u32) {
        if self.spsel {
            self.set_sp(v);
        } else {
            self.banked_sp = v;
        }
    }
    pub fn sp_main(&self) -> u32 {
        if !self.spsel {
            self.sp()
        } else {
            self.banked_sp
        }
    }
    pub fn set_sp_main(&mut self, v: u32) {
        if !self.spsel {
            self.set_sp(v);
        } else {
            self.banked_sp = v;
        }
    }

    pub fn pendsv_priority(&self) -> i32 {
        ((self.shpr3 >> 22) & 0x3) as i32
    }
    pub fn svcall_priority(&self) -> i32 {
        (self.shpr2 >> 30) as i32
    }
    pub fn systick_priority(&self) -> i32 {
        (self.shpr3 >> 30) as i32
    }

    pub fn exception_priority(&self, n: u32) -> i32 {
        match n {
            EXC_RESET => -3,
            EXC_NMI => -2,
            EXC_HARDFAULT => -1,
            EXC_SVCALL => self.svcall_priority(),
            EXC_PENDSV => self.pendsv_priority(),
            EXC_SYSTICK => self.systick_priority(),
            _ => {
                if n < 16 {
                    return LOWEST_PRIORITY;
                }
                // JavaScript masks a shift count to five bits.
                let bit = 1u32 << (n.wrapping_sub(16) & 31);
                for p in 0..4 {
                    if self.interrupt_priorities[p] & bit != 0 {
                        return p as i32;
                    }
                }
                LOWEST_PRIORITY
            }
        }
    }

    /// `vectPending`: the ICSR field. It reads the pending set unmasked by
    /// the enable set, as the reference does.
    pub fn vect_pending(&self) -> u32 {
        if self.pending_nmi {
            return EXC_NMI;
        }
        let (svp, stp, psp) = (self.svcall_priority(), self.systick_priority(), self.pendsv_priority());
        for priority in 0..LOWEST_PRIORITY {
            let level = self.pending_interrupts & self.interrupt_priorities[priority as usize];
            if self.pending_svcall && priority == svp {
                return EXC_SVCALL;
            }
            if self.pending_pendsv && priority == psp {
                return EXC_PENDSV;
            }
            if self.pending_systick && priority == stp {
                return EXC_SYSTICK;
            }
            if level != 0 {
                return 16 + level.trailing_zeros();
            }
        }
        0
    }

    /// `substractUpdateFlags`, on exact integers as the reference's doubles.
    pub fn sub_flags(&mut self, minuend: i64, subtrahend: i64) -> u32 {
        let result = minuend - subtrahend;
        let rb = result as u32;
        let res31 = rb & 0x8000_0000 != 0;
        let m31 = (minuend as u32) & 0x8000_0000 != 0;
        let s31 = (subtrahend as u32) & 0x8000_0000 != 0;
        self.n = res31;
        self.z = rb == 0;
        self.c = minuend >= subtrahend;
        self.v = (res31 && !m31 && s31) || (!res31 && m31 && !s31);
        rb
    }

    /// `addUpdateFlags`.
    pub fn add_flags(&mut self, a: i64, b: i64) -> u32 {
        let result = a + b;
        let unsigned = result as u32;
        let signed = (a as u32 as i32 as i64) + (b as u32 as i32 as i64);
        self.n = unsigned & 0x8000_0000 != 0;
        self.z = unsigned == 0;
        self.c = result != unsigned as i64;
        self.v = (unsigned as i32 as i64) != signed;
        unsigned
    }

    fn set_nz(&mut self, v: u32) {
        self.n = v & 0x8000_0000 != 0;
        self.z = v == 0;
    }
}

fn sign_extend8(v: u32) -> u32 {
    ((v << 24) as i32 >> 24) as u32
}
fn sign_extend16(v: u32) -> u32 {
    ((v << 16) as i32 >> 16) as u32
}

/// `cyclesIO`: the extra cycles a load or store costs by region.
pub fn cycles_io(addr: u32, write: bool) -> u32 {
    if (SIO_START..SIO_START + 0x1000_0000).contains(&addr) {
        return 0;
    }
    if (APB_START..APB_START + 0x1000_0000).contains(&addr) {
        return if write { 4 } else { 3 };
    }
    1
}

impl Rp2040 {
    /// `CortexM0Core.reset`.
    pub fn core_reset(&mut self) {
        let vtor = self.cpu.vtor;
        let sp = self.read_u32(vtor);
        self.cpu.set_sp(sp);
        let pc = self.read_u32(vtor.wrapping_add(4));
        self.cpu.r[PC] = pc & 0xffff_fffe;
        self.cpu.cycles = 0;
    }

    /// `exceptionEntry`.
    pub fn exception_entry(&mut self, exception_number: u32) {
        let frame_ptr;
        let frame_ptr_align;
        if self.cpu.spsel && !self.cpu.handler_mode {
            frame_ptr_align = if self.cpu.sp_process() & 0b100 != 0 { 1 } else { 0 };
            let v = self.cpu.sp_process().wrapping_sub(0x20) & !0b100;
            self.cpu.set_sp_process(v);
            frame_ptr = self.cpu.sp_process();
        } else {
            frame_ptr_align = if self.cpu.sp_main() & 0b100 != 0 { 1 } else { 0 };
            let v = self.cpu.sp_main().wrapping_sub(0x20) & !0b100;
            self.cpu.set_sp_main(v);
            frame_ptr = self.cpu.sp_main();
        }
        let r = self.cpu.r;
        self.write_u32(frame_ptr, r[0]);
        self.write_u32(frame_ptr.wrapping_add(0x4), r[1]);
        self.write_u32(frame_ptr.wrapping_add(0x8), r[2]);
        self.write_u32(frame_ptr.wrapping_add(0xc), r[3]);
        self.write_u32(frame_ptr.wrapping_add(0x10), r[12]);
        self.write_u32(frame_ptr.wrapping_add(0x14), self.cpu.r[LR]);
        self.write_u32(frame_ptr.wrapping_add(0x18), self.cpu.r[PC] & !1);
        let xpsr = (self.cpu.xpsr() & !(1 << 9)) | (frame_ptr_align << 9);
        self.write_u32(frame_ptr.wrapping_add(0x1c), xpsr);
        if self.cpu.handler_mode {
            self.cpu.r[LR] = 0xffff_fff1;
        } else if !self.cpu.spsel {
            self.cpu.r[LR] = 0xffff_fff9;
        } else {
            self.cpu.r[LR] = 0xffff_fffd;
        }
        self.cpu.handler_mode = true;
        self.cpu.ipsr = exception_number;
        self.cpu.switch_stack(false);
        self.cpu.event_registered = true;
        let vector = self.cpu.vtor.wrapping_add(exception_number.wrapping_mul(4));
        self.cpu.r[PC] = self.read_u32(vector);
    }

    /// `exceptionReturn`.
    pub fn exception_return(&mut self, exc_return: u32) {
        let mut frame_ptr = self.cpu.sp_main();
        match exc_return & 0xf {
            0b0001 => {
                self.cpu.handler_mode = true;
                self.cpu.switch_stack(false);
            }
            0b1001 => {
                self.cpu.handler_mode = false;
                self.cpu.switch_stack(false);
            }
            0b1101 => {
                frame_ptr = self.cpu.sp_process();
                self.cpu.handler_mode = false;
                self.cpu.switch_stack(true);
            }
            _ => {}
        }
        self.cpu.r[0] = self.read_u32(frame_ptr);
        self.cpu.r[1] = self.read_u32(frame_ptr.wrapping_add(0x4));
        self.cpu.r[2] = self.read_u32(frame_ptr.wrapping_add(0x8));
        self.cpu.r[3] = self.read_u32(frame_ptr.wrapping_add(0xc));
        self.cpu.r[12] = self.read_u32(frame_ptr.wrapping_add(0x10));
        self.cpu.r[LR] = self.read_u32(frame_ptr.wrapping_add(0x14));
        self.cpu.r[PC] = self.read_u32(frame_ptr.wrapping_add(0x18));
        let psr = self.read_u32(frame_ptr.wrapping_add(0x1c));
        let align = if psr & (1 << 9) != 0 { 0b100 } else { 0 };
        match exc_return & 0xf {
            0b0001 | 0b1001 => {
                let v = self.cpu.sp_main().wrapping_add(0x20) | align;
                self.cpu.set_sp_main(v);
            }
            0b1101 => {
                let v = self.cpu.sp_process().wrapping_add(0x20) | align;
                self.cpu.set_sp_process(v);
            }
            _ => {}
        }
        self.cpu.set_apsr(psr & 0xf000_0000);
        let force_thread = !self.cpu.handler_mode && self.cpu.npriv;
        self.cpu.ipsr = if force_thread { 0 } else { psr & 0x3f };
        self.cpu.interrupts_updated = true;
        self.cpu.event_registered = true;
    }

    /// `setInterrupt`: a peripheral's IRQ line.
    pub fn set_interrupt(&mut self, irq: u32, value: bool) {
        let bit = 1u32 << (irq & 31);
        if value && self.cpu.pending_interrupts & bit == 0 {
            self.cpu.pending_interrupts |= bit;
            self.cpu.interrupts_updated = true;
            if self.cpu.waiting && self.check_for_interrupts() {
                self.cpu.waiting = false;
            }
        } else if !value {
            self.cpu.pending_interrupts &= !bit;
        }
    }

    /// `checkForInterrupts`: takes the highest-priority pending exception
    /// that preempts, if any.
    pub fn check_for_interrupts(&mut self) -> bool {
        let cpu = &self.cpu;
        let current_priority = if cpu.waiting {
            if cpu.pm {
                cpu.exception_priority(cpu.ipsr)
            } else {
                LOWEST_PRIORITY
            }
        } else {
            cpu.exception_priority(cpu.ipsr).min(if cpu.pm { 0 } else { LOWEST_PRIORITY })
        };
        let interrupt_set = cpu.pending_interrupts & cpu.enabled_interrupts;
        let (svp, stp, psp) = (cpu.svcall_priority(), cpu.systick_priority(), cpu.pendsv_priority());
        if self.cpu.pending_nmi {
            self.cpu.pending_nmi = false;
            self.exception_entry(EXC_NMI);
            return true;
        }
        let mut priority = 0;
        while priority < current_priority {
            let level = interrupt_set & self.cpu.interrupt_priorities[priority as usize];
            if self.cpu.pending_svcall && priority == svp {
                self.cpu.pending_svcall = false;
                self.exception_entry(EXC_SVCALL);
                return true;
            }
            if self.cpu.pending_pendsv && priority == psp {
                self.cpu.pending_pendsv = false;
                self.exception_entry(EXC_PENDSV);
                return true;
            }
            if self.cpu.pending_systick && priority == stp {
                self.cpu.pending_systick = false;
                self.exception_entry(EXC_SYSTICK);
                return true;
            }
            if level != 0 {
                let n = level.trailing_zeros();
                if n > MAX_HARDWARE_IRQ {
                    self.cpu.pending_interrupts &= !(1u32 << n);
                }
                self.exception_entry(16 + n);
                return true;
            }
            priority += 1;
        }
        self.cpu.interrupts_updated = false;
        false
    }

    /// `readSpecialRegister`.
    pub fn read_special_register(&mut self, sysm: u32) -> u32 {
        match sysm {
            SYSM_APSR => self.cpu.apsr(),
            SYSM_XPSR => self.cpu.xpsr(),
            SYSM_IPSR => self.cpu.ipsr,
            SYSM_PRIMASK => self.cpu.pm as u32,
            SYSM_MSP => self.cpu.sp_main(),
            SYSM_PSP => self.cpu.sp_process(),
            SYSM_CONTROL => (if self.cpu.spsel { 2 } else { 0 }) | (self.cpu.npriv as u32),
            _ => {
                self.warnings += 1;
                0
            }
        }
    }

    /// `writeSpecialRegister`.
    pub fn write_special_register(&mut self, sysm: u32, value: u32) {
        match sysm {
            SYSM_APSR => self.cpu.set_apsr(value),
            SYSM_XPSR => self.cpu.set_xpsr(value),
            SYSM_IPSR => self.cpu.ipsr = value,
            SYSM_PRIMASK => {
                self.cpu.pm = value & 1 != 0;
                self.cpu.interrupts_updated = true;
            }
            SYSM_MSP => self.cpu.set_sp_main(value),
            SYSM_PSP => self.cpu.set_sp_process(value),
            SYSM_CONTROL => {
                self.cpu.npriv = value & 1 != 0;
                if !self.cpu.handler_mode {
                    self.cpu.switch_stack(value & 2 != 0);
                }
            }
            _ => self.warnings += 1,
        }
    }

    /// `BXWritePC`.
    pub fn bx_write_pc(&mut self, address: u32) {
        if self.cpu.handler_mode && address >> 28 == 0b1111 {
            self.exception_return(address & 0x0fff_ffff);
        } else {
            self.cpu.r[PC] = address & !1;
        }
    }

    fn on_break(&mut self, code: u32) {
        self.halt(Stop::Break { code });
    }

    /// `executeInstruction`: runs one instruction and returns its cycles.
    pub fn execute_instruction(&mut self) -> u32 {
        if self.cpu.interrupts_updated && self.check_for_interrupts() {
            self.cpu.waiting = false;
        }
        let opcode_pc = self.cpu.r[PC] & !1;
        let opcode = self.read_u16(opcode_pc);
        let wide = opcode >> 12 == 0b1111 || opcode >> 11 == 0b11101;
        let opcode2 = if wide { self.read_u16(opcode_pc.wrapping_add(2)) } else { 0 };
        self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
        let mut delta: u32 = 1;
        let lo3 = |x: u32| (x & 0x7) as usize;

        // ADCS
        if opcode >> 6 == 0b0100000101 {
            let rm = lo3(opcode >> 3);
            let rdn = lo3(opcode);
            let a = self.cpu.r[rm] as i64;
            let b = self.cpu.r[rdn] as i64 + self.cpu.c as i64;
            self.cpu.r[rdn] = self.cpu.add_flags(a, b);
        }
        // ADD (register = SP plus immediate)
        else if opcode >> 11 == 0b10101 {
            let imm8 = opcode & 0xff;
            let rd = lo3(opcode >> 8);
            self.cpu.r[rd] = self.cpu.sp().wrapping_add(imm8 << 2);
        }
        // ADD (SP plus immediate)
        else if opcode >> 7 == 0b101100000 {
            let imm32 = (opcode & 0x7f) << 2;
            let v = self.cpu.sp().wrapping_add(imm32);
            self.cpu.set_sp(v);
        }
        // ADDS (encoding T1)
        else if opcode >> 9 == 0b0001110 {
            let imm3 = (opcode >> 6) & 0x7;
            let rn = lo3(opcode >> 3);
            let rd = lo3(opcode);
            self.cpu.r[rd] = self.cpu.add_flags(self.cpu.r[rn] as i64, imm3 as i64);
        }
        // ADDS (encoding T2)
        else if opcode >> 11 == 0b00110 {
            let imm8 = opcode & 0xff;
            let rdn = lo3(opcode >> 8);
            self.cpu.r[rdn] = self.cpu.add_flags(self.cpu.r[rdn] as i64, imm8 as i64);
        }
        // ADDS (register)
        else if opcode >> 9 == 0b0001100 {
            let rm = lo3(opcode >> 6);
            let rn = lo3(opcode >> 3);
            let rd = lo3(opcode);
            self.cpu.r[rd] = self.cpu.add_flags(self.cpu.r[rn] as i64, self.cpu.r[rm] as i64);
        }
        // ADD (register)
        else if opcode >> 8 == 0b01000100 {
            let rm = ((opcode >> 3) & 0xf) as usize;
            let rdn = (((opcode & 0x80) >> 4) | (opcode & 0x7)) as usize;
            let left = if rdn == PC { self.cpu.r[PC].wrapping_add(2) } else { self.cpu.r[rdn] };
            let right = if rm == PC { self.cpu.r[PC].wrapping_add(2) } else { self.cpu.r[rm] };
            let result = left.wrapping_add(right);
            if rdn != SP && rdn != PC {
                self.cpu.r[rdn] = result;
            } else if rdn == PC {
                self.cpu.r[rdn] = result & !1;
                delta += 1;
            } else {
                self.cpu.r[rdn] = result & !3;
            }
        }
        // ADR
        else if opcode >> 11 == 0b10100 {
            let imm8 = opcode & 0xff;
            let rd = lo3(opcode >> 8);
            self.cpu.r[rd] = (opcode_pc & 0xffff_fffc).wrapping_add(4).wrapping_add(imm8 << 2);
        }
        // ANDS (encoding T2)
        else if opcode >> 6 == 0b0100000000 {
            let rm = lo3(opcode >> 3);
            let rdn = lo3(opcode);
            let result = self.cpu.r[rdn] & self.cpu.r[rm];
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
        }
        // ASRS (immediate)
        else if opcode >> 11 == 0b00010 {
            let imm5 = (opcode >> 6) & 0x1f;
            let rm = lo3(opcode >> 3);
            let rd = lo3(opcode);
            let input = self.cpu.r[rm];
            let shift_n = if imm5 != 0 { imm5 } else { 32 };
            let result = if shift_n < 32 { ((input as i32) >> shift_n) as u32 } else { ((input as i32) >> 31) as u32 };
            self.cpu.r[rd] = result;
            self.cpu.set_nz(result);
            self.cpu.c = (input >> ((shift_n - 1) & 31)) & 1 != 0;
        }
        // ASRS (register)
        else if opcode >> 6 == 0b0100000100 {
            let rm = lo3(opcode >> 3);
            let rdn = lo3(opcode);
            let input = self.cpu.r[rdn];
            let amount = self.cpu.r[rm] & 0xff;
            let shift_n = if amount < 32 { amount } else { 32 };
            let result = if shift_n < 32 { ((input as i32) >> shift_n) as u32 } else { ((input as i32) >> 31) as u32 };
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
            // Reference quirk: a shift of 0 tests bit (0 - 1) & 31 = 31
            // instead of leaving C unchanged.
            self.cpu.c = (input >> (shift_n.wrapping_sub(1) & 31)) & 1 != 0;
        }
        // B (with condition)
        else if opcode >> 12 == 0b1101 && ((opcode >> 9) & 0x7) != 0b111 {
            let mut imm8 = ((opcode & 0xff) << 1) as i32;
            let cond = (opcode >> 8) & 0xf;
            if imm8 & (1 << 8) != 0 {
                imm8 = (imm8 & 0x1ff) - 0x200;
            }
            if self.cpu.check_condition(cond) {
                self.cpu.r[PC] = self.cpu.r[PC].wrapping_add((imm8 + 2) as u32);
                delta += 1;
            }
        }
        // B
        else if opcode >> 11 == 0b11100 {
            let mut imm11 = ((opcode & 0x7ff) << 1) as i32;
            if imm11 & (1 << 11) != 0 {
                imm11 = (imm11 & 0x7ff) - 0x800;
            }
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add((imm11 + 2) as u32);
            delta += 1;
        }
        // BICS
        else if opcode >> 6 == 0b0100001110 {
            let rm = lo3(opcode >> 3);
            let rdn = lo3(opcode);
            let result = self.cpu.r[rdn] & !self.cpu.r[rm];
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
        }
        // BKPT
        else if opcode >> 8 == 0b10111110 {
            let imm8 = opcode & 0xff;
            self.cpu.break_rewind = 2;
            self.on_break(imm8);
        }
        // BL
        else if opcode >> 11 == 0b11110 && opcode2 >> 14 == 0b11 && ((opcode2 >> 12) & 0x1) == 1 {
            let imm11 = opcode2 & 0x7ff;
            let j2 = (opcode2 >> 11) & 0x1;
            let j1 = (opcode2 >> 13) & 0x1;
            let imm10 = opcode & 0x3ff;
            let s = (opcode >> 10) & 0x1;
            let i1 = 1 - (s ^ j1);
            let i2 = 1 - (s ^ j2);
            let imm32 = (if s != 0 { 0xffu32 << 24 } else { 0 }) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
            self.cpu.r[LR] = self.cpu.r[PC].wrapping_add(2) | 1;
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2).wrapping_add(imm32);
            delta += 2;
        }
        // BLX
        else if opcode >> 7 == 0b010001111 && (opcode & 0x7) == 0 {
            let rm = ((opcode >> 3) & 0xf) as usize;
            self.cpu.r[LR] = self.cpu.r[PC] | 1;
            self.cpu.r[PC] = self.cpu.r[rm] & !1;
            delta += 1;
        }
        // BX
        else if opcode >> 7 == 0b010001110 && (opcode & 0x7) == 0 {
            let rm = ((opcode >> 3) & 0xf) as usize;
            let target = self.cpu.r[rm];
            self.bx_write_pc(target);
            delta += 1;
        }
        // CMN (register)
        else if opcode >> 6 == 0b0100001011 {
            let rm = lo3(opcode >> 3);
            let rn = lo3(opcode);
            self.cpu.add_flags(self.cpu.r[rn] as i64, self.cpu.r[rm] as i64);
        }
        // CMP (immediate)
        else if opcode >> 11 == 0b00101 {
            let rn = lo3(opcode >> 8);
            let imm8 = opcode & 0xff;
            self.cpu.sub_flags(self.cpu.r[rn] as i64, imm8 as i64);
        }
        // CMP (register)
        else if opcode >> 6 == 0b0100001010 {
            let rm = lo3(opcode >> 3);
            let rn = lo3(opcode);
            self.cpu.sub_flags(self.cpu.r[rn] as i64, self.cpu.r[rm] as i64);
        }
        // CMP (register), encoding T2
        else if opcode >> 8 == 0b01000101 {
            let rm = ((opcode >> 3) & 0xf) as usize;
            let rn = (((opcode >> 4) & 0x8) | (opcode & 0x7)) as usize;
            self.cpu.sub_flags(self.cpu.r[rn] as i64, self.cpu.r[rm] as i64);
        }
        // CPSID i
        else if opcode == 0xb672 {
            self.cpu.pm = true;
        }
        // CPSIE i
        else if opcode == 0xb662 {
            self.cpu.pm = false;
            self.cpu.interrupts_updated = true;
        }
        // DMB SY
        else if opcode == 0xf3bf && (opcode2 & 0xfff0) == 0x8f50 {
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
            delta += 2;
        }
        // DSB SY
        else if opcode == 0xf3bf && (opcode2 & 0xfff0) == 0x8f40 {
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
            delta += 2;
        }
        // EORS
        else if opcode >> 6 == 0b0100000001 {
            let rm = lo3(opcode >> 3);
            let rdn = lo3(opcode);
            let result = self.cpu.r[rm] ^ self.cpu.r[rdn];
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
        }
        // ISB SY
        else if opcode == 0xf3bf && (opcode2 & 0xfff0) == 0x8f60 {
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
            delta += 2;
        }
        // LDMIA
        else if opcode >> 11 == 0b11001 {
            let rn = lo3(opcode >> 8);
            let registers = opcode & 0xff;
            let mut address = self.cpu.r[rn];
            for i in 0..8 {
                if registers & (1 << i) != 0 {
                    self.cpu.r[i] = self.read_u32(address);
                    address = address.wrapping_add(4);
                    delta += 1;
                }
            }
            if registers & (1 << rn) == 0 {
                self.cpu.r[rn] = address;
            }
        }
        // LDR (immediate)
        else if opcode >> 11 == 0b01101 {
            let imm5 = ((opcode >> 6) & 0x1f) << 2;
            let rn = lo3(opcode >> 3);
            let rt = lo3(opcode);
            let addr = self.cpu.r[rn].wrapping_add(imm5);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u32(addr);
        }
        // LDR (SP plus immediate)
        else if opcode >> 11 == 0b10011 {
            let rt = lo3(opcode >> 8);
            let imm8 = opcode & 0xff;
            let addr = self.cpu.sp().wrapping_add(imm8 << 2);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u32(addr);
        }
        // LDR (literal)
        else if opcode >> 11 == 0b01001 {
            let imm8 = (opcode & 0xff) << 2;
            let rt = lo3(opcode >> 8);
            let next_pc = self.cpu.r[PC].wrapping_add(2);
            let addr = (next_pc & 0xffff_fffc).wrapping_add(imm8);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u32(addr);
        }
        // LDR (register)
        else if opcode >> 9 == 0b0101100 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u32(addr);
        }
        // LDRB (immediate)
        else if opcode >> 11 == 0b01111 {
            let imm5 = (opcode >> 6) & 0x1f;
            let (rn, rt) = (lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rn].wrapping_add(imm5);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u8(addr);
        }
        // LDRB (register)
        else if opcode >> 9 == 0b0101110 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u8(addr);
        }
        // LDRH (immediate)
        else if opcode >> 11 == 0b10001 {
            let imm5 = (opcode >> 6) & 0x1f;
            let (rn, rt) = (lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rn].wrapping_add(imm5 << 1);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u16(addr);
        }
        // LDRH (register)
        else if opcode >> 9 == 0b0101101 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = self.read_u16(addr);
        }
        // LDRSB
        else if opcode >> 9 == 0b0101011 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = sign_extend8(self.read_u8(addr));
        }
        // LDRSH
        else if opcode >> 9 == 0b0101111 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let addr = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(addr, false);
            self.cpu.r[rt] = sign_extend16(self.read_u16(addr));
        }
        // LSLS (immediate)
        else if opcode >> 11 == 0b00000 {
            let imm5 = (opcode >> 6) & 0x1f;
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rm];
            let result = input << imm5;
            self.cpu.r[rd] = result;
            self.cpu.set_nz(result);
            if imm5 != 0 {
                self.cpu.c = (input >> (32 - imm5)) & 1 != 0;
            }
        }
        // LSLS (register)
        else if opcode >> 6 == 0b0100000010 {
            let (rm, rdn) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rdn];
            let shift = self.cpu.r[rm] & 0xff;
            let result = if shift >= 32 { 0 } else { input << shift };
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
            if shift != 0 {
                // Reference quirk: past 32 the tested bit is (32 - n) & 31
                // instead of C = 0.
                self.cpu.c = (input >> (32u32.wrapping_sub(shift) & 31)) & 1 != 0;
            }
        }
        // LSRS (immediate)
        else if opcode >> 11 == 0b00001 {
            let imm5 = (opcode >> 6) & 0x1f;
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rm];
            let result = if imm5 != 0 { input >> imm5 } else { 0 };
            self.cpu.r[rd] = result;
            self.cpu.set_nz(result);
            self.cpu.c = (input >> (if imm5 != 0 { imm5 - 1 } else { 31 })) & 1 != 0;
        }
        // LSRS (register)
        else if opcode >> 6 == 0b0100000011 {
            let (rm, rdn) = (lo3(opcode >> 3), lo3(opcode));
            let shift = self.cpu.r[rm] & 0xff;
            let input = self.cpu.r[rdn];
            let result = if shift < 32 { input >> shift } else { 0 };
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
            // Reference quirk: a shift of 0 tests bit 31 instead of leaving
            // C unchanged.
            self.cpu.c = shift <= 32 && (input >> (shift.wrapping_sub(1) & 31)) & 1 != 0;
        }
        // MOV
        else if opcode >> 8 == 0b01000110 {
            let rm = ((opcode >> 3) & 0xf) as usize;
            let rd = (((opcode >> 4) & 0x8) | (opcode & 0x7)) as usize;
            let mut value = if rm == PC { self.cpu.r[PC].wrapping_add(2) } else { self.cpu.r[rm] };
            if rd == PC {
                delta += 1;
                value &= !1;
            } else if rd == SP {
                value &= !3;
            }
            self.cpu.r[rd] = value;
        }
        // MOVS
        else if opcode >> 11 == 0b00100 {
            let value = opcode & 0xff;
            let rd = lo3(opcode >> 8);
            self.cpu.r[rd] = value;
            self.cpu.set_nz(value);
        }
        // MRS
        else if opcode == 0b1111001111101111 && opcode2 >> 12 == 0b1000 {
            let sysm = opcode2 & 0xff;
            let rd = ((opcode2 >> 8) & 0xf) as usize;
            self.cpu.r[rd] = self.read_special_register(sysm);
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
            delta += 2;
        }
        // MSR
        else if opcode >> 4 == 0b111100111000 && opcode2 >> 8 == 0b10001000 {
            let sysm = opcode2 & 0xff;
            let rn = (opcode & 0xf) as usize;
            let value = self.cpu.r[rn];
            self.write_special_register(sysm, value);
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
            delta += 2;
        }
        // MULS
        else if opcode >> 6 == 0b0100001101 {
            let (rn, rdm) = (lo3(opcode >> 3), lo3(opcode));
            let result = self.cpu.r[rn].wrapping_mul(self.cpu.r[rdm]);
            self.cpu.r[rdm] = result;
            self.cpu.set_nz(result);
        }
        // MVNS
        else if opcode >> 6 == 0b0100001111 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            let result = !self.cpu.r[rm];
            self.cpu.r[rd] = result;
            self.cpu.set_nz(result);
        }
        // ORRS (encoding T2)
        else if opcode >> 6 == 0b0100001100 {
            let (rm, rdn) = (lo3(opcode >> 3), lo3(opcode));
            let result = self.cpu.r[rdn] | self.cpu.r[rm];
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
        }
        // POP
        else if opcode >> 9 == 0b1011110 {
            let p = (opcode >> 8) & 1;
            let mut address = self.cpu.sp();
            for i in 0..=7 {
                if opcode & (1 << i) != 0 {
                    self.cpu.r[i] = self.read_u32(address);
                    address = address.wrapping_add(4);
                    delta += 1;
                }
            }
            if p != 0 {
                self.cpu.set_sp(address.wrapping_add(4));
                let target = self.read_u32(address);
                self.bx_write_pc(target);
                delta += 2;
            } else {
                self.cpu.set_sp(address);
            }
        }
        // PUSH
        else if opcode >> 9 == 0b1011010 {
            let bit_count = (opcode & 0x1ff).count_ones();
            let mut address = self.cpu.sp().wrapping_sub(4 * bit_count);
            for i in 0..=7 {
                if opcode & (1 << i) != 0 {
                    let v = self.cpu.r[i];
                    self.write_u32(address, v);
                    delta += 1;
                    address = address.wrapping_add(4);
                }
            }
            if opcode & (1 << 8) != 0 {
                let v = self.cpu.r[LR];
                self.write_u32(address, v);
            }
            let v = self.cpu.sp().wrapping_sub(4 * bit_count);
            self.cpu.set_sp(v);
        }
        // REV
        else if opcode >> 6 == 0b1011101000 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.r[rm].swap_bytes();
        }
        // REV16
        else if opcode >> 6 == 0b1011101001 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rm];
            self.cpu.r[rd] = (((input >> 16) & 0xff) << 24)
                | (((input >> 24) & 0xff) << 16)
                | ((input & 0xff) << 8)
                | ((input >> 8) & 0xff);
        }
        // REVSH
        else if opcode >> 6 == 0b1011101011 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rm];
            self.cpu.r[rd] = sign_extend16(((input & 0xff) << 8) | ((input >> 8) & 0xff));
        }
        // ROR
        else if opcode >> 6 == 0b0100000111 {
            let (rm, rdn) = (lo3(opcode >> 3), lo3(opcode));
            let input = self.cpu.r[rdn];
            let shift = (self.cpu.r[rm] & 0xff) % 32;
            let result = input.rotate_right(shift);
            self.cpu.r[rdn] = result;
            self.cpu.set_nz(result);
            // Reference quirk: C takes bit 31 of the result even when the
            // rotate amount (Rm[7:0]) is 0.
            self.cpu.c = result & 0x8000_0000 != 0;
        }
        // NEGS / RSBS
        else if opcode >> 6 == 0b0100001001 {
            let (rn, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.sub_flags(0, self.cpu.r[rn] as i64);
        }
        // NOP
        else if opcode == 0b1011111100000000 {
        }
        // SBCS (encoding T1)
        else if opcode >> 6 == 0b0100000110 {
            let (rm, rdn) = (lo3(opcode >> 3), lo3(opcode));
            let sub = self.cpu.r[rm] as i64 + (1 - self.cpu.c as i64);
            self.cpu.r[rdn] = self.cpu.sub_flags(self.cpu.r[rdn] as i64, sub);
        }
        // SEV
        else if opcode == 0b1011111101000000 {
        }
        // STMIA
        else if opcode >> 11 == 0b11000 {
            let rn = lo3(opcode >> 8);
            let registers = opcode & 0xff;
            let mut address = self.cpu.r[rn];
            for i in 0..8 {
                if registers & (1 << i) != 0 {
                    let v = self.cpu.r[i];
                    self.write_u32(address, v);
                    address = address.wrapping_add(4);
                    delta += 1;
                }
            }
            // Reference quirk: no write-back when Rn is in the list.
            if registers & (1 << rn) == 0 {
                self.cpu.r[rn] = address;
            }
        }
        // STR (immediate)
        else if opcode >> 11 == 0b01100 {
            let imm5 = ((opcode >> 6) & 0x1f) << 2;
            let (rn, rt) = (lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rn].wrapping_add(imm5);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u32(address, v);
        }
        // STR (SP plus immediate)
        else if opcode >> 11 == 0b10010 {
            let rt = lo3(opcode >> 8);
            let imm8 = opcode & 0xff;
            let address = self.cpu.sp().wrapping_add(imm8 << 2);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u32(address, v);
        }
        // STR (register)
        else if opcode >> 9 == 0b0101000 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u32(address, v);
        }
        // STRB (immediate)
        else if opcode >> 11 == 0b01110 {
            let imm5 = (opcode >> 6) & 0x1f;
            let (rn, rt) = (lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rn].wrapping_add(imm5);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u8(address, v);
        }
        // STRB (register)
        else if opcode >> 9 == 0b0101010 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u8(address, v);
        }
        // STRH (immediate)
        else if opcode >> 11 == 0b10000 {
            let imm5 = ((opcode >> 6) & 0x1f) << 1;
            let (rn, rt) = (lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rn].wrapping_add(imm5);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u16(address, v);
        }
        // STRH (register)
        else if opcode >> 9 == 0b0101001 {
            let (rm, rn, rt) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            let address = self.cpu.r[rm].wrapping_add(self.cpu.r[rn]);
            delta += cycles_io(address, true);
            let v = self.cpu.r[rt];
            self.write_u16(address, v);
        }
        // SUB (SP minus immediate)
        else if opcode >> 7 == 0b101100001 {
            let imm32 = (opcode & 0x7f) << 2;
            let v = self.cpu.sp().wrapping_sub(imm32);
            self.cpu.set_sp(v);
        }
        // SUBS (encoding T1)
        else if opcode >> 9 == 0b0001111 {
            let imm3 = (opcode >> 6) & 0x7;
            let (rn, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.sub_flags(self.cpu.r[rn] as i64, imm3 as i64);
        }
        // SUBS (encoding T2)
        else if opcode >> 11 == 0b00111 {
            let imm8 = opcode & 0xff;
            let rdn = lo3(opcode >> 8);
            self.cpu.r[rdn] = self.cpu.sub_flags(self.cpu.r[rdn] as i64, imm8 as i64);
        }
        // SUBS (register)
        else if opcode >> 9 == 0b0001101 {
            let (rm, rn, rd) = (lo3(opcode >> 6), lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.sub_flags(self.cpu.r[rn] as i64, self.cpu.r[rm] as i64);
        }
        // SVC
        else if opcode >> 8 == 0b11011111 {
            self.cpu.pending_svcall = true;
            self.cpu.interrupts_updated = true;
        }
        // SXTB
        else if opcode >> 6 == 0b1011001001 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = sign_extend8(self.cpu.r[rm]);
        }
        // SXTH
        else if opcode >> 6 == 0b1011001000 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = sign_extend16(self.cpu.r[rm]);
        }
        // TST
        else if opcode >> 6 == 0b0100001000 {
            let (rm, rn) = (lo3(opcode >> 3), lo3(opcode));
            let result = self.cpu.r[rn] & self.cpu.r[rm];
            self.cpu.set_nz(result);
        }
        // UDF
        else if opcode >> 8 == 0b11011110 {
            let imm8 = opcode & 0xff;
            self.cpu.break_rewind = 2;
            self.on_break(imm8);
        }
        // UDF (encoding T2)
        else if opcode >> 4 == 0b111101111111 && opcode2 >> 12 == 0b1010 {
            let imm4 = opcode & 0xf;
            let imm12 = opcode2 & 0xfff;
            self.cpu.break_rewind = 4;
            self.on_break((imm4 << 12) | imm12);
            self.cpu.r[PC] = self.cpu.r[PC].wrapping_add(2);
        }
        // UXTB
        else if opcode >> 6 == 0b1011001011 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.r[rm] & 0xff;
        }
        // UXTH
        else if opcode >> 6 == 0b1011001010 {
            let (rm, rd) = (lo3(opcode >> 3), lo3(opcode));
            self.cpu.r[rd] = self.cpu.r[rm] & 0xffff;
        }
        // WFE
        else if opcode == 0b1011111100100000 {
            delta += 1;
            if self.cpu.event_registered {
                self.cpu.event_registered = false;
            } else {
                self.cpu.waiting = true;
            }
        }
        // WFI
        else if opcode == 0b1011111100110000 {
            delta += 1;
            self.cpu.waiting = true;
        }
        // YIELD
        else if opcode == 0b1011111100010000 {
        } else {
            // Unimplemented: the reference logs two warnings and moves on
            // (PC already advanced by 2 only, even for a 32-bit encoding).
            self.warnings += 1;
        }
        self.cpu.cycles += delta as u64;
        delta
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Cpu {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.banked_sp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cycles, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.event_registered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.waiting, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.n, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.z, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.c, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.break_rewind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pm, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.spsel, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.npriv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.handler_mode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ipsr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_interrupts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.enabled_interrupts, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interrupt_priorities, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_nmi, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_pendsv, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_svcall, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pending_systick, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interrupts_updated, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vtor, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shpr2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shpr3, w); }
        n
    }
}
