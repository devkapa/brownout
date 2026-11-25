//! Source-level debugging for a board: brownout's `ArduinoMcu` breakpoints,
//! one-shot step targets and pause, so a host folding onto the core keeps the
//! same contract (`MicrocontrollerCore.setBreakpoints`, `stepTo`, `pausedAt`,
//! `resume`, `debugSnapshot`, `readData`). Not a port of avr8js, which has
//! none of it. Addresses are program byte addresses, as DWARF line tables
//! give them; a board pauses before running the instruction there.

use std::collections::HashSet;

use super::cpu::Cpu;
use super::instr::avr_instruction;

/// The ATmega328P's data space (RAMEND 0x08FF): reads stop there even when
/// the core was built with more SRAM, as `ArduinoMcu.readData` does.
pub const DATA_SPACE_BYTES: usize = 0x900;

#[derive(Default)]
pub struct Debugger {
    pub breakpoints: HashSet<u32>,
    step_stops: Option<HashSet<u32>>,
    step_min_sp: u16,
    paused_at: Option<u32>,
}

impl Debugger {
    pub fn set_breakpoints(&mut self, addresses: &[u32]) {
        self.breakpoints = addresses.iter().copied().collect();
    }

    /// Resumes, then pauses at the first of `addresses` reached with SP at or
    /// above `min_sp` (one source line, over calls or into them). One-shot.
    pub fn step_to(&mut self, addresses: &[u32], min_sp: u16) {
        self.step_stops = Some(addresses.iter().copied().collect());
        self.step_min_sp = min_sp;
        self.resume();
    }

    pub fn paused_at(&self) -> Option<u32> {
        self.paused_at
    }

    pub fn resume(&mut self) {
        self.paused_at = None;
    }

    /// A board reset ends a pause and a step; breakpoints stay.
    pub fn on_reset(&mut self) {
        self.paused_at = None;
        self.step_stops = None;
    }

    /// `ArduinoMcu.step`'s instruction loop for `cycles` cycles, with the
    /// pause checks. Returns false when paused (nothing ran, or it stopped
    /// early); the caller drops the rest of the time rather than owing it.
    pub fn run(&mut self, cpu: &mut Cpu, cycles: u64) -> bool {
        if self.paused_at.is_some() {
            return false;
        }
        let end = cpu.cycles + cycles;
        let mut first = true;
        while cpu.cycles < end {
            if !first && (!self.breakpoints.is_empty() || self.step_stops.is_some()) {
                let pc = (cpu.pc as u32) * 2;
                let step_hit = self.step_stops.as_ref().is_some_and(|s| s.contains(&pc)) && cpu.sp() >= self.step_min_sp;
                if self.breakpoints.contains(&pc) || step_hit {
                    self.paused_at = Some(pc);
                    self.step_stops = None;
                    return false;
                }
            }
            first = false;
            avr_instruction(cpu);
            cpu.tick();
        }
        true
    }
}

impl Cpu {
    /// `ArduinoMcu.readData`: a copy of data space, cut short at RAMEND.
    pub fn read_data_range(&self, address: usize, length: usize) -> Vec<u8> {
        let end = self.data.len().min(DATA_SPACE_BYTES);
        let start = address.min(end);
        self.data[start..end.min(start.saturating_add(length))].to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::super::periph_tests::{board, SERIAL_COUNTER};
    use super::*;

    /// Where a free-running serial_counter is after `cycles`.
    fn landmark(cycles: u64) -> (u32, u64) {
        let mut cpu = board(SERIAL_COUNTER);
        Debugger::default().run(&mut cpu, cycles);
        ((cpu.pc as u32) * 2, cpu.cycles)
    }

    #[test]
    fn a_breakpoint_pauses_before_its_instruction_and_a_paused_board_runs_nothing() {
        let (pc, cycles) = landmark(3_200_000);
        let mut cpu = board(SERIAL_COUNTER);
        let mut dbg = Debugger::default();
        dbg.set_breakpoints(&[pc]);
        assert!(!dbg.run(&mut cpu, 3_200_000));
        assert_eq!(dbg.paused_at(), Some(pc));
        assert_eq!((cpu.pc as u32) * 2, pc);
        assert!(cpu.cycles <= cycles);
        let at = cpu.cycles;
        assert!(!dbg.run(&mut cpu, 800_000));
        assert_eq!(cpu.cycles, at);
    }

    #[test]
    fn a_resumed_board_passes_its_breakpoint_and_stops_there_next_time_round() {
        let (pc, _) = landmark(3_200_000);
        let mut cpu = board(SERIAL_COUNTER);
        let mut dbg = Debugger::default();
        dbg.set_breakpoints(&[pc]);
        dbg.run(&mut cpu, 3_200_000);
        let first = cpu.cycles;
        dbg.resume();
        dbg.run(&mut cpu, 3_200_000);
        assert_eq!(dbg.paused_at(), Some(pc));
        assert!(cpu.cycles > first);
    }

    #[test]
    fn a_step_stops_at_or_above_its_stack_floor_only_and_is_one_shot() {
        let (pc, _) = landmark(3_200_000);
        let mut cpu = board(SERIAL_COUNTER);
        let mut dbg = Debugger::default();
        dbg.set_breakpoints(&[pc]);
        dbg.run(&mut cpu, 3_200_000);
        let sp = cpu.sp();
        dbg.set_breakpoints(&[]);

        let mut below = board(SERIAL_COUNTER);
        let mut d2 = Debugger::default();
        d2.set_breakpoints(&[pc]);
        d2.run(&mut below, 3_200_000);
        d2.set_breakpoints(&[]);
        d2.step_to(&[pc], sp + 1);
        assert!(d2.run(&mut below, 3_200_000), "no stop below the floor");

        dbg.step_to(&[pc], sp);
        dbg.run(&mut cpu, 3_200_000);
        assert_eq!(dbg.paused_at(), Some(pc));
        assert!(cpu.sp() >= sp);
        dbg.resume();
        assert!(dbg.run(&mut cpu, 3_200_000), "a step is one-shot");
    }

    #[test]
    fn reads_stop_at_ramend() {
        let mut cpu = board(SERIAL_COUNTER);
        Debugger::default().run(&mut cpu, 800_000);
        let io = cpu.read_data_range(0x5d, 3);
        assert_eq!(u16::from_le_bytes([io[0], io[1]]), cpu.sp());
        assert_eq!(io[2], cpu.sreg());
        assert_eq!(cpu.read_data_range(0x8fe, 8).len(), 2);
        assert!(cpu.read_data_range(0x2000, 4).is_empty());
    }

    #[test]
    fn a_reset_ends_a_pause_but_keeps_breakpoints() {
        let (pc, _) = landmark(3_200_000);
        let mut cpu = board(SERIAL_COUNTER);
        let mut dbg = Debugger::default();
        dbg.set_breakpoints(&[pc]);
        dbg.run(&mut cpu, 3_200_000);
        dbg.on_reset();
        assert_eq!(dbg.paused_at(), None);
        assert!(dbg.breakpoints.contains(&pc));
    }
}
