//! The machine the lock-step harness drives: the CPU plus the host inputs
//! the harness applies at fixed cycle counts (plan-1.0.0 section 4.3). AVR1
//! has no peripherals, so the only input is raising an interrupt flag, which
//! is how a timer or pin would reach the CPU; AVR2's ports, timers and ADC
//! extend this with pin levels and ADC values.

use super::cpu::{Cpu, DEFAULT_SRAM_BYTES};
use super::instr::avr_instruction;
use super::interrupt::Interrupt;

/// Words in the record `state_record` writes.
pub const STATE_WORDS: usize = 42;

/// An interrupt raised (`setInterruptFlag`) before the first instruction that
/// starts at or after `at_cycle`, on both sides of the harness.
#[derive(Clone, Copy, Debug)]
pub struct ScheduledIrq {
    pub at_cycle: u64,
    pub interrupt: Interrupt,
}

pub struct Machine {
    pub cpu: Cpu,
    pub irqs: Vec<ScheduledIrq>,
    pub steps: u64,
}

impl Machine {
    pub fn new(prog: Vec<u16>) -> Self {
        Machine { cpu: Cpu::new(prog, DEFAULT_SRAM_BYTES), irqs: Vec::new(), steps: 0 }
    }

    fn apply_due_irqs(&mut self) {
        let mut i = 0;
        while i < self.irqs.len() {
            if self.irqs[i].at_cycle <= self.cpu.cycles {
                let irq = self.irqs.remove(i);
                self.cpu.set_interrupt_flag(irq.interrupt);
            } else {
                i += 1;
            }
        }
    }

    /// One iteration of `ArduinoMcu.step`'s loop: `avrInstruction`, then
    /// `tick`.
    pub fn step(&mut self) {
        if !self.irqs.is_empty() {
            self.apply_due_irqs();
        }
        avr_instruction(&mut self.cpu);
        self.cpu.tick();
        self.steps += 1;
    }

    /// Runs up to `max_steps` instructions; stops after the one that made
    /// the first peripheral access when `io_trap` is on. Returns the count.
    pub fn run(&mut self, max_steps: u64) -> u64 {
        let start = self.steps;
        while self.steps - start < max_steps && self.cpu.io_hit.is_none() {
            self.step();
        }
        self.steps - start
    }
}

/// 32-bit FNV-1a over `bytes`, the RAM hash both sides of the harness use.
pub fn fnv1a(bytes: &[u8]) -> u32 {
    let mut h: u32 = 0x811c9dc5;
    for &b in bytes {
        h ^= b as u32;
        h = h.wrapping_mul(0x01000193);
    }
    h
}

/// The compared state: 0 PC (words), 1 SREG, 2 SP, 3 cycles, 4 next pending
/// interrupt (-1 none), 5 first peripheral access address (-1 none), 6 its
/// write flag, 7 hash of the whole data space, 8 clock events pending,
/// 9 watchdog resets, 10..42 r0..r31.
pub fn state_record(cpu: &Cpu, out: &mut [f64]) {
    out.iter_mut().for_each(|w| *w = 0.0);
    out[0] = cpu.pc as f64;
    out[1] = cpu.sreg() as f64;
    out[2] = cpu.sp() as f64;
    out[3] = cpu.cycles as f64;
    out[4] = cpu.next_interrupt as f64;
    out[5] = cpu.io_hit.map_or(-1.0, |h| h.address as f64);
    out[6] = cpu.io_hit.map_or(0.0, |h| h.write as u8 as f64);
    out[7] = fnv1a(&cpu.data) as f64;
    out[8] = cpu.clock_events.len() as f64;
    out[9] = cpu.watchdog_resets as f64;
    for i in 0..32 {
        out[10 + i] = cpu.data[i] as f64;
    }
}
