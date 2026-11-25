//! EEPROM, ported from avr8js 0.21.0 `peripherals/eeprom.js` (MIT, Copyright
//! (c) Uri Shaked and contributors; see NOTICE): `AVREEPROM` with
//! `eepromConfig` and an `EEPROMMemoryBackend`, whose memory is `memory`
//! here. A write without erase only clears bits (`memory &= value`), an erase
//! sets the byte to 0xFF. The host carries `memory` from board to board, as
//! `ArduinoMcu` keeps one backend across `reset()`.

use super::cpu::{Cpu, EV_EEMPE_CLEAR, EV_EEPROM_READY};
use super::interrupt::Interrupt;

pub const EECR: usize = 0x3f;
pub const EEDR: usize = 0x40;
pub const EEARL: usize = 0x41;
pub const EEARH: usize = 0x42;
pub const EEPROM_BYTES: usize = 1024;

/// 1.8 ms at 16 MHz each.
const ERASE_CYCLES: u64 = 28800;
const WRITE_CYCLES: u64 = 28800;

const EERE: u8 = 1 << 0;
const EEPE: u8 = 1 << 1;
const EEMPE: u8 = 1 << 2;
const EERIE: u8 = 1 << 3;
const EEPM0: u8 = 1 << 4;
const EEPM1: u8 = 1 << 5;
const EECR_WRITE_MASK: u8 = EEPE | EEMPE | EERIE | EEPM0 | EEPM1;

pub struct Eeprom {
    pub memory: Vec<u8>,
    /// Until this cycle, setting EEPE starts a write (EEMPE's four cycles).
    pub write_enabled_cycles: u64,
    /// Until this cycle a write is in progress.
    pub write_complete_cycles: u64,
    pub interrupt: Interrupt,
}

impl Eeprom {
    pub fn new(memory: Vec<u8>) -> Eeprom {
        Eeprom {
            memory,
            write_enabled_cycles: 0,
            write_complete_cycles: 0,
            interrupt: Interrupt {
                address: 0x2c,
                flag_register: EECR as u16,
                flag_mask: EEPE,
                enable_register: EECR as u16,
                enable_mask: EERIE,
                constant: true,
                inverse_flag: true,
            },
        }
    }
}

impl Cpu {
    fn eeprom(&mut self) -> &mut Eeprom {
        self.p.eeprom.as_mut().unwrap()
    }

    pub fn eeprom_memory(&self) -> &[u8] {
        self.p.eeprom.as_ref().map_or(&[], |e| &e.memory)
    }

    pub fn eeprom_memory_mut(&mut self) -> &mut [u8] {
        &mut self.eeprom().memory
    }

    /// The clock event EEMPE schedules: its four cycles are over.
    pub(crate) fn eeprom_eempe_event(&mut self) {
        self.data[EECR] &= !EEMPE;
    }

    /// The clock event a write schedules: it has finished (EEPE clears, the
    /// flag being active low).
    pub(crate) fn eeprom_ready_event(&mut self) {
        let i = self.eeprom().interrupt;
        self.set_interrupt_flag(i);
    }

    /// `writeHooks[EECR]`: always consumed. An address past the memory reads
    /// 0 and drops writes (the reference's `Uint8Array` out of range).
    pub(crate) fn hook_eecr(&mut self, eecr: u8) -> bool {
        let addr = ((self.data[EEARH] as usize) << 8) | self.data[EEARL] as usize;
        self.data[EECR] = (self.data[EECR] & !EECR_WRITE_MASK) | (eecr & EECR_WRITE_MASK);
        let i = self.eeprom().interrupt;
        self.update_interrupt_enable(i, eecr);
        if eecr & EERE != 0 {
            self.clear_interrupt(i, true);
        }
        if eecr & EEMPE != 0 {
            let eempe_cycles = 4;
            let at = self.cycles + eempe_cycles;
            self.eeprom().write_enabled_cycles = at;
            self.add_clock_event(EV_EEMPE_CLEAR, eempe_cycles as i64);
        }
        if eecr & EERE != 0 {
            self.data[EEDR] = self.eeprom().memory.get(addr).copied().unwrap_or(0);
            // The CPU halts four cycles after a read.
            self.cycles += 4;
            return true;
        }
        if eecr & EEPE != 0 {
            let cycles = self.cycles;
            let e = self.eeprom();
            if cycles >= e.write_enabled_cycles {
                self.data[EECR] &= !EEPE;
                return true;
            }
            if cycles < e.write_complete_cycles {
                return true;
            }
            let eedr = self.data[EEDR];
            let e = self.eeprom();
            e.write_complete_cycles = cycles;
            if eecr & EEPM1 == 0 {
                if let Some(m) = e.memory.get_mut(addr) {
                    *m = 0xff;
                }
                e.write_complete_cycles += ERASE_CYCLES;
            }
            if eecr & EEPM0 == 0 {
                if let Some(m) = e.memory.get_mut(addr) {
                    *m &= eedr;
                }
                e.write_complete_cycles += WRITE_CYCLES;
            }
            let delay = e.write_complete_cycles as i64 - cycles as i64;
            self.data[EECR] |= EEPE;
            self.add_clock_event(EV_EEPROM_READY, delay);
            // The CPU halts two cycles after setting EEPE.
            self.cycles += 2;
        }
        true
    }
}
