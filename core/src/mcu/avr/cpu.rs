//! ATmega328P CPU state, ported from avr8js 0.21.0 `cpu/cpu.js` (MIT,
//! Copyright (c) Uri Shaked and contributors; see NOTICE).
//!
//! The state is plain arrays: `data` is the register file, I/O space and
//! SRAM in one byte array (registers 0x00-0x1f, I/O 0x20-0xff, SRAM from
//! 0x100), SP and SREG live in it at 0x5d-0x5f exactly as in the reference.
//! The reference's `CPU(progMem, sramBytes = 8192)` is what brownout's
//! `ArduinoMcu` constructs, so the default here is 8192 SRAM bytes (the
//! ATmega328P really has 2048): a stack that runs past 0x8ff behaves
//! identically in lock-step.
//!
//! Where JavaScript would read `undefined` out of a typed array (an index
//! past the end of `data`, a program word past the flash), this port reads
//! 0 and drops writes; the reference's NaN program counters after such an
//! access are not modelled (no sketch gets there with a sane SP).

use super::adc::Adc;
use super::eeprom::Eeprom;
use super::gpio::{Port, PortEvent};
use super::interrupt::{avr_interrupt, Interrupt};
use super::spi::Spi;
use super::timer::Timer;
use super::twi::Twi;
use super::usart::Usart;

pub const REGISTER_SPACE: usize = 0x100;
pub const MAX_INTERRUPTS: usize = 128;
pub const DEFAULT_SRAM_BYTES: usize = 8192;

/// Data-space addresses of the core registers that live inside the I/O
/// window (RAMPZ, EIND, SPL, SPH, SREG). They are the CPU's own, not a
/// peripheral's.
pub const ADDR_RAMPZ: usize = 0x5b;
pub const ADDR_EIND: usize = 0x5c;
pub const ADDR_SP: usize = 0x5d;
pub const ADDR_SREG: usize = 0x5f;

/// A pending clock event (`addClockEvent`). The reference stores a callback;
/// the port stores an id the owner of the peripherals maps back to one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockEvent {
    pub cycles: u64,
    pub id: u32,
    /// What the reference's callback closed over (the ADC's result).
    pub arg: u32,
}

/// First id of the events the peripherals schedule; below it ids are the
/// caller's own (the CPU tests use small ones with `tick_with`).
pub const EV_BASE: u32 = 0x1000;
pub const EV_TIMER: u32 = EV_BASE; // + timer index
pub const EV_ADC_READ: u32 = EV_BASE + 0x10;
pub const EV_USART_TX: u32 = EV_BASE + 0x20;
pub const EV_USART_RX: u32 = EV_BASE + 0x21; // arg: the byte
pub const EV_SPI_DONE: u32 = EV_BASE + 0x22; // arg: the byte read back
pub const EV_TWI: u32 = EV_BASE + 0x23; // arg: TWCR | status << 8 | TWDR << 16
pub const EV_EEMPE_CLEAR: u32 = EV_BASE + 0x24;
pub const EV_EEPROM_READY: u32 = EV_BASE + 0x25;
pub const EV_TWI_DONE: u32 = EV_BASE + 0x26; // arg: TWI_DONE_* (twi.rs)

/// Peripheral state. Empty until `attach_arduino_peripherals`; the hook
/// tables are indexed by data address and hold `(kind << 8) | index`, 0 for
/// no hook (the reference's `readHooks`/`writeHooks`).
#[derive(Default)]
pub struct Periph {
    pub attached: bool,
    pub hook_w: Vec<u16>,
    pub hook_r: Vec<u16>,
    pub ports: Vec<Port>,
    pub timers: Vec<Timer>,
    pub adc: Option<Adc>,
    pub usart: Option<Usart>,
    pub spi: Option<Spi>,
    pub twi: Option<Twi>,
    pub eeprom: Option<Eeprom>,
    /// Bus traffic for protocol decoders (see `bus`).
    pub bus: super::bus::BusLog,
    /// Every GPIO write-listener call since the host last drained this.
    pub events: Vec<PortEvent>,
    /// Whether the write listener is installed (`addListener`/`removeListener`).
    pub listening: bool,
    /// Test support: record the timers' pin overrides and the ADC's mux inputs
    /// (what the upstream suites spy on).
    pub log_overrides: bool,
    pub overrides: Vec<(usize, u8, super::gpio::PinOverride)>,
    pub adc_reads: Vec<super::adc::MuxInput>,
}

/// The first access to a peripheral register (data address 0x20-0xff outside
/// the CPU's own registers) since `io_trap` was switched on. The lock-step
/// harness stops there: AVR1 has no peripherals, so past it the reference
/// and the port would only be comparing plain memory.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IoHit {
    pub address: u16,
    pub write: bool,
}

pub struct Cpu {
    /// Program memory, little-endian 16-bit words.
    pub prog: Vec<u16>,
    /// Registers, I/O space and SRAM.
    pub data: Vec<u8>,
    /// The program counter in words. Signed because the reference holds -1
    /// transiently (`JMP 0` stores `k - 1`) and lets relative jumps leave
    /// the flash before the `% length` wrap.
    pub pc: i32,
    pub cycles: u64,
    pub pc22_bits: bool,
    pub pending_interrupts: [Option<Interrupt>; MAX_INTERRUPTS],
    pub next_interrupt: i32,
    pub max_interrupt: usize,
    /// Sorted by `cycles`; a new event goes before any with an equal time.
    pub clock_events: Vec<ClockEvent>,
    pub io_trap: bool,
    pub io_hit: Option<IoHit>,
    /// Count of `WDR` instructions (the reference calls `onWatchdogReset`).
    pub watchdog_resets: u64,
    pub p: Periph,
}

impl Cpu {
    pub fn new(prog: Vec<u16>, sram_bytes: usize) -> Self {
        let pc22_bits = prog.len() * 2 > 0x20000;
        let mut cpu = Cpu {
            prog,
            data: vec![0; sram_bytes + REGISTER_SPACE],
            pc: 0,
            cycles: 0,
            pc22_bits,
            pending_interrupts: [None; MAX_INTERRUPTS],
            next_interrupt: -1,
            max_interrupt: 0,
            clock_events: Vec::new(),
            io_trap: false,
            io_hit: None,
            watchdog_resets: 0,
            p: Periph::default(),
        };
        cpu.reset();
        cpu
    }

    /// `CPU.reset()`: SP to the last byte of data space, PC to 0, interrupts
    /// and clock events dropped. Registers, SRAM and the cycle counter are
    /// left alone, as in the reference.
    pub fn reset(&mut self) {
        let top = (self.data.len() - 1) as u16;
        self.set_sp(top);
        self.pc = 0;
        self.pending_interrupts = [None; MAX_INTERRUPTS];
        self.next_interrupt = -1;
        self.clock_events.clear();
    }

    // ---- raw access -----------------------------------------------------

    /// A byte of data space, 0 past the end (JavaScript's `undefined`).
    #[inline]
    pub fn rd(&self, i: i32) -> u8 {
        if i >= 0 && (i as usize) < self.data.len() {
            self.data[i as usize]
        } else {
            0
        }
    }

    /// Stores a byte, dropping it past the end.
    #[inline]
    pub fn wr(&mut self, i: i32, v: u8) {
        if i >= 0 && (i as usize) < self.data.len() {
            self.data[i as usize] = v;
        }
    }

    #[inline]
    pub fn get16(&self, i: usize) -> u16 {
        u16::from_le_bytes([self.data[i], self.data[i + 1]])
    }

    #[inline]
    pub fn set16(&mut self, i: usize, v: u16) {
        let b = v.to_le_bytes();
        self.data[i] = b[0];
        self.data[i + 1] = b[1];
    }

    #[inline]
    pub fn sp(&self) -> u16 {
        self.get16(ADDR_SP)
    }

    #[inline]
    pub fn set_sp(&mut self, v: u16) {
        self.set16(ADDR_SP, v)
    }

    #[inline]
    pub fn sreg(&self) -> u8 {
        self.data[ADDR_SREG]
    }

    pub fn interrupts_enabled(&self) -> bool {
        self.sreg() & 0x80 != 0
    }

    /// `progMem[i]`; `None` is the reference's `undefined`.
    #[inline]
    pub fn word(&self, i: i32) -> Option<u16> {
        if i >= 0 {
            self.prog.get(i as usize).copied()
        } else {
            None
        }
    }

    /// `progBytes[i]` (the byte view of program memory), 0 past the end.
    #[inline]
    pub fn prog_byte(&self, i: u32) -> u8 {
        let w = self.prog.get((i >> 1) as usize).copied().unwrap_or(0);
        if i & 1 == 0 {
            w as u8
        } else {
            (w >> 8) as u8
        }
    }

    pub fn prog_len_bytes(&self) -> usize {
        self.prog.len() * 2
    }

    // ---- data space with the peripheral hook point ------------------------

    #[inline]
    fn peripheral_address(addr: i32) -> bool {
        (0x20..0x100).contains(&addr) && !(ADDR_RAMPZ as i32..=ADDR_SREG as i32).contains(&addr)
    }

    /// Whether `addr` is a peripheral register no ported peripheral models
    /// (what `io_trap` reports once peripherals are attached).
    pub fn unported_peripheral(&self, addr: i32) -> bool {
        Self::peripheral_address(addr) && !(self.p.attached && super::gpio::is_ported_register(addr as u16))
    }

    /// `readData`: addresses below 32 skip the read hooks.
    #[inline]
    pub fn read_data(&mut self, addr: i32) -> u8 {
        if self.io_trap && addr >= 32 && self.io_hit.is_none() && self.unported_peripheral(addr) {
            self.io_hit = Some(IoHit { address: addr as u16, write: false });
        }
        if addr >= 32 && self.p.attached {
            let code = self.p.hook_r.get(addr as usize).copied().unwrap_or(0);
            if code != 0 {
                return self.run_read_hook(code, addr as usize);
            }
        }
        self.rd(addr)
    }

    /// `writeData` with the reference's default mask.
    #[inline]
    pub fn write_data(&mut self, addr: i32, value: u8) {
        self.write_data_mask(addr, value, 0xff);
    }

    /// `writeData(addr, value, mask)`: a write hook that returns true keeps
    /// the plain store from happening.
    pub fn write_data_mask(&mut self, addr: i32, value: u8, mask: u8) {
        if self.io_trap && self.io_hit.is_none() && self.unported_peripheral(addr) {
            self.io_hit = Some(IoHit { address: addr as u16, write: true });
        }
        if addr >= 0 && self.p.attached {
            let code = self.p.hook_w.get(addr as usize).copied().unwrap_or(0);
            if code != 0 {
                let old = self.rd(addr);
                if self.run_write_hook(code, value, old, addr as usize, mask) {
                    return;
                }
            }
        }
        self.wr(addr, value);
    }

    // ---- interrupts -------------------------------------------------------

    pub fn set_interrupt_flag(&mut self, interrupt: Interrupt) {
        let fr = interrupt.flag_register as usize;
        if interrupt.inverse_flag {
            self.data[fr] &= !interrupt.flag_mask;
        } else {
            self.data[fr] |= interrupt.flag_mask;
        }
        if self.data[interrupt.enable_register as usize] & interrupt.enable_mask != 0 {
            self.queue_interrupt(interrupt);
        }
    }

    pub fn update_interrupt_enable(&mut self, interrupt: Interrupt, register_value: u8) {
        if register_value & interrupt.enable_mask != 0 {
            let bit_set = self.data[interrupt.flag_register as usize] & interrupt.flag_mask != 0;
            if if interrupt.inverse_flag { !bit_set } else { bit_set } {
                self.queue_interrupt(interrupt);
            }
        } else {
            self.clear_interrupt(interrupt, false);
        }
    }

    pub fn queue_interrupt(&mut self, interrupt: Interrupt) {
        let address = interrupt.address as usize;
        self.pending_interrupts[address] = Some(interrupt);
        if self.next_interrupt == -1 || self.next_interrupt > address as i32 {
            self.next_interrupt = address as i32;
        }
        if address > self.max_interrupt {
            self.max_interrupt = address;
        }
    }

    pub fn clear_interrupt(&mut self, interrupt: Interrupt, clear_flag: bool) {
        let address = interrupt.address as usize;
        if clear_flag {
            self.data[interrupt.flag_register as usize] &= !interrupt.flag_mask;
        }
        if self.pending_interrupts[address].is_none() {
            return;
        }
        self.pending_interrupts[address] = None;
        if self.next_interrupt == address as i32 {
            self.next_interrupt = -1;
            for i in address + 1..=self.max_interrupt {
                if self.pending_interrupts[i].is_some() {
                    self.next_interrupt = i as i32;
                    break;
                }
            }
        }
    }

    pub fn clear_interrupt_by_flag(&mut self, interrupt: Interrupt, register_value: u8) {
        if register_value & interrupt.flag_mask != 0 {
            self.data[interrupt.flag_register as usize] &= !interrupt.flag_mask;
            self.clear_interrupt(interrupt, true);
        }
    }

    // ---- clock events -----------------------------------------------------

    /// `addClockEvent(callback, cycles)`: due `max(1, cycles)` from now.
    pub fn add_clock_event(&mut self, id: u32, cycles: i64) {
        self.add_clock_event_arg(id, 0, cycles);
    }

    pub fn add_clock_event_arg(&mut self, id: u32, arg: u32, cycles: i64) {
        let due = self.cycles + cycles.max(1) as u64;
        let at = self.clock_events.iter().position(|e| e.cycles >= due).unwrap_or(self.clock_events.len());
        self.clock_events.insert(at, ClockEvent { cycles: due, id, arg });
    }

    /// `clearClockEvent`: drops the first event with this id.
    pub fn clear_clock_event(&mut self, id: u32) -> bool {
        match self.clock_events.iter().position(|e| e.id == id) {
            Some(i) => {
                self.clock_events.remove(i);
                true
            }
            None => false,
        }
    }

    /// `updateClockEvent`: re-times an event that exists.
    pub fn update_clock_event(&mut self, id: u32, cycles: i64) -> bool {
        if self.clear_clock_event(id) {
            self.add_clock_event(id, cycles);
            true
        } else {
            false
        }
    }

    /// `tick()`: at most one due clock event fires (its id goes to
    /// `on_event`, which may add events), then at most one pending interrupt
    /// is taken. Call it after every instruction.
    pub fn tick_with(&mut self, mut on_event: impl FnMut(&mut Cpu, u32)) {
        self.tick_events(|cpu, ev| on_event(cpu, ev.id));
    }

    /// `tick()` with the whole event, so a handler can read its argument.
    pub fn tick_events(&mut self, mut on_event: impl FnMut(&mut Cpu, ClockEvent)) {
        if let Some(&first) = self.clock_events.first() {
            if first.cycles <= self.cycles {
                self.clock_events.remove(0);
                on_event(self, first);
            }
        }
        let next = self.next_interrupt;
        if self.interrupts_enabled() && next >= 0 {
            if let Some(interrupt) = self.pending_interrupts[next as usize] {
                avr_interrupt(self, interrupt.address);
                if !interrupt.constant {
                    self.clear_interrupt(interrupt, true);
                }
            }
        }
    }

    /// `tick()`: due clock events go to the peripherals.
    pub fn tick(&mut self) {
        self.tick_events(|cpu, ev| cpu.dispatch_event(ev));
    }
}
