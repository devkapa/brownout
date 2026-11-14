//! RP2040 emulator, ported from rp2040js 1.3.3 (MIT, Uri Shaked and
//! contributors; see NOTICE). Upstream: `src/rp2040.ts` (the machine and the
//! memory map) and `src/simulator.ts` (the step loop).
//!
//! Package R1 (plan-1.0.0 section 4.2): the Cortex-M0+ core (`core.rs`), the
//! bus and memory map (`bus.rs`), the PPB with the NVIC and SysTick
//! (`ppb.rs`), and SIO with the divider, spinlocks and interpolators
//! (`sio.rs`, `interp.rs`). The clock and Timer32 (`clock.rs`) are the parts
//! of rp2040js's clock and timer utilities SysTick needs.
//!
//! The port is behaviour-exact against the reference, quirks included: the
//! lock-step harness (`scripts/rp2040-lockstep.mjs`) holds it to zero
//! divergence, so where rp2040js departs from ARMv6-M the port departs the
//! same way, and the unit tests name each such place (`tests.rs`,
//! "reference quirk").
//!
//! Package R2 (the small system blocks every boot touches): clocks, xosc,
//! resets, psm, tbman, syscfg, sysinfo, busctrl and the unimplemented stubs
//! (`periph.rs`), the timer and watchdog (`timer.rs`), the RTC (`rtc.rs`)
//! and the IO and pad banks (`gpiobank.rs`).
//!
//! Every peripheral not yet ported (SSI, the UARTs, SPI, I2C, ADC, PWM, DMA,
//! PIO, the USB controller and its DPRAM hook) halts the machine at its
//! first access with a [`Stop`] naming it, instead of guessing at its
//! behaviour.

pub mod abi;
pub mod adc;
pub mod bus;
pub mod clock;
pub mod core;
pub mod dma;
pub mod gpiobank;
pub mod interp;
pub mod periph;
pub mod pio;
pub mod ppb;
pub mod pwm;
pub mod rtc;
pub mod sio;
pub mod spi;
pub mod ssi;
pub mod timer;
pub mod uart;
pub mod usb;

#[cfg(test)]
mod r2_tests;
#[cfg(test)]
mod r2_upstream_tests;
#[cfg(test)]
mod r3_tests;
#[cfg(test)]
mod r4_tests;
#[cfg(test)]
mod r5_tests;
#[cfg(test)]
mod r6_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod upstream_tests;

use self::clock::Clock;
use self::core::Cpu;
use self::periph::Periph;
use self::ppb::Ppb;
use self::sio::Sio;

pub const BOOTROM_WORDS: usize = 4 * 1024;
pub const SRAM_BYTES: usize = 264 * 1024;
pub const FLASH_BYTES: usize = 16 * 1024 * 1024;
pub const DPRAM_BYTES: usize = 4 * 1024;

/// rp2040js's Simulator: 125 MHz, so one cycle is exactly 8 ns.
pub const CYCLE_NANOS: f64 = 8.0;

/// Why the machine stopped. Everything except `Break` and `Idle` cuts the
/// instruction that hit it short (the reference either throws there or runs
/// code R1 does not port): the step does not count, the core is put back as
/// it was before that instruction, and the clock does not advance. Memory
/// the instruction wrote before the stop keeps the write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// First access to a peripheral R1 does not port.
    Peripheral { name: &'static str, address: u32, write: bool },
    /// A write to the USB DPRAM, which calls the USB controller (R4).
    UsbDpram { address: u32 },
    /// A misaligned 32-bit read: rp2040js's logger throws on it.
    UnalignedRead { address: u32 },
    /// A DataView access past the end of a memory: rp2040js throws a
    /// RangeError there.
    OutOfRange { address: u32 },
    /// BKPT or UDF: the reference calls `onBreak`, which the Simulator
    /// treats as stop. The instruction completed.
    Break { code: u32 },
    /// WFI/WFE with no alarm left to wake it: the reference would spin on a
    /// zero-length tick forever. Nothing executed.
    Idle,
}

impl Stop {
    /// Whether the step that hit this stop completed as the reference's did.
    pub fn step_completed(&self) -> bool {
        matches!(self, Stop::Break { .. } | Stop::Idle)
    }
}

/// The write a halting peripheral access would have made: the register
/// offset within the block, the atomic alias (0 normal, 1 XOR, 2 SET,
/// 3 CLEAR) and the 32-bit value the reference passes on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PeripheralWrite {
    pub offset: u32,
    pub atomic: u32,
    pub value: u32,
}

/// A host input applied at a fixed cycle count (clock nanos / 8), before the
/// first step that starts at or after it, on both sides of the harness.
#[derive(Clone, Copy, Debug)]
pub struct ScheduledInput {
    pub at_cycle: f64,
    pub pin: u32,
    pub level: bool,
}

/// REPL bytes the host feeds the CDC at a fixed cycle count, like a pin input.
#[derive(Clone, Debug)]
pub struct ScheduledRepl {
    pub at_cycle: f64,
    pub bytes: Vec<u8>,
}

/// One change of a GPIO's drive state, as rp2040js's pin listeners see it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinEvent {
    pub pin: u32,
    pub state: u32,
    pub nanos: f64,
}

pub struct Rp2040 {
    pub cpu: Cpu,
    pub bootrom: Vec<u32>,
    pub sram: Vec<u8>,
    pub flash: Vec<u8>,
    /// 4 KB flash pages written through the bus or a loader since the last
    /// flash hash; the harness hashes only these between full hashes.
    pub flash_dirty: std::collections::BTreeSet<u32>,
    pub dpram: Vec<u8>,
    pub clock: Clock,
    pub ppb: Ppb,
    pub sio: Sio,
    pub periph: Periph,
    pub stop: Option<Stop>,
    pub peripheral_write: Option<PeripheralWrite>,
    /// Count of accesses the reference only logs a warning for (invalid
    /// addresses, unimplemented opcodes). Not part of the compared state.
    pub warnings: u64,
    pub steps: u64,
    pub inputs: Vec<ScheduledInput>,
    pub repl_inputs: Vec<ScheduledRepl>,
    pub pin_events: Vec<PinEvent>,
    /// When non-zero, [`Rp2040::pio_host_turn`] runs after every this-many
    /// completed steps: the stand-in for the host event loop the harness
    /// uses on both sides (0: the host calls it itself).
    pub host_turn_steps: u64,
}

impl Default for Rp2040 {
    fn default() -> Self {
        Self::new()
    }
}

impl Rp2040 {
    /// `new RP2040()`: every block constructed, then `reset()`, which reads
    /// the vectors from the (still empty) boot ROM.
    pub fn new() -> Self {
        let mut m = Rp2040 {
            cpu: Cpu::new(),
            bootrom: vec![0; BOOTROM_WORDS],
            sram: vec![0; SRAM_BYTES],
            flash: vec![0; FLASH_BYTES],
            flash_dirty: Default::default(),
            dpram: vec![0; DPRAM_BYTES],
            clock: Clock::new(),
            ppb: Ppb::new(),
            sio: Sio::new(),
            periph: Periph::new(),
            stop: None,
            peripheral_write: None,
            warnings: 0,
            steps: 0,
            inputs: Vec::new(),
            repl_inputs: Vec::new(),
            pin_events: Vec::new(),
            host_turn_steps: 0,
        };
        m.dma_reset_channels();
        m.ppb_construct();
        m.reset();
        // Nothing listens to the pins while the reference constructs itself.
        m.pin_events.clear();
        m
    }

    /// `RP2040.reset()`: the core reloads SP and PC from the vector table
    /// and zeroes its cycle count, the PWM block resets, and flash is erased.
    pub fn reset(&mut self) {
        self.core_reset();
        // The real PWM reset. Its visible side effect on the pins is the
        // reference's GPIO1 quirk (see pwm.rs).
        self.pwm_reset();
        self.flash.fill(0xff);
    }

    /// `RP2040.loadBootrom()`.
    pub fn load_bootrom(&mut self, words: &[u32]) {
        let n = words.len().min(BOOTROM_WORDS);
        self.bootrom[..n].copy_from_slice(&words[..n]);
        self.reset();
    }

    pub(crate) fn halt(&mut self, stop: Stop) {
        if self.stop.is_none() {
            self.stop = Some(stop);
        }
    }

    /// Applies every scheduled input due at the current clock cycle, in the
    /// order they were scheduled.
    fn apply_due_inputs(&mut self) {
        if self.inputs.is_empty() && self.repl_inputs.is_empty() {
            return;
        }
        let now_cycle = self.clock.nanos / CYCLE_NANOS;
        let mut i = 0;
        while i < self.inputs.len() {
            if self.inputs[i].at_cycle <= now_cycle {
                let inp = self.inputs.remove(i);
                self.set_input_value(inp.pin, inp.level);
            } else {
                i += 1;
            }
        }
        let mut i = 0;
        while i < self.repl_inputs.len() {
            if self.repl_inputs[i].at_cycle <= now_cycle {
                let inp = self.repl_inputs.remove(i);
                self.cdc_send(&inp.bytes);
            } else {
                i += 1;
            }
        }
    }

    /// The earliest cycle any scheduled input is due at.
    fn next_input_cycle(&self) -> Option<f64> {
        self.inputs
            .iter()
            .map(|i| i.at_cycle)
            .chain(self.repl_inputs.iter().map(|i| i.at_cycle))
            .fold(None, |m: Option<f64>, c| Some(m.map_or(c, |m| m.min(c))))
    }

    /// One iteration of rp2040js's `Simulator.execute` loop: a waiting core
    /// fast-forwards the clock to the next alarm; otherwise one instruction
    /// runs and the clock advances by its cycles.
    pub fn step(&mut self) {
        self.apply_due_inputs();
        if self.cpu.waiting {
            if self.clock.alarms.is_empty() {
                // Nothing is scheduled, but a host input still to come wakes
                // the machine: the clock jumps to it (the wrapper idles its
                // time budget out the same way) and the next step applies it.
                match self.next_input_cycle() {
                    Some(c) if c * CYCLE_NANOS > self.clock.nanos => {
                        let dn = c * CYCLE_NANOS - self.clock.nanos;
                        self.clock_tick(dn);
                        self.finish_step();
                        return;
                    }
                    _ => {
                        self.halt(Stop::Idle);
                        return;
                    }
                }
            }
            let dn = self.clock.nanos_to_next_alarm();
            self.clock_tick(dn);
        } else {
            // A stop that cuts an instruction short leaves the core as it was
            // before it, so the machine halts on a state the reference also
            // reaches (it would go on to run that instruction).
            let before = self.cpu.clone();
            let cycles = self.execute_instruction();
            if let Some(stop) = self.stop {
                if !stop.step_completed() {
                    self.cpu = before;
                    return;
                }
            }
            self.clock_tick(cycles as f64 * CYCLE_NANOS);
        }
        self.finish_step();
    }

    /// Counts a completed step and, when the host turn is automatic, takes
    /// it at the step boundary.
    fn finish_step(&mut self) {
        self.steps += 1;
        if self.host_turn_steps != 0 && self.steps % self.host_turn_steps == 0 {
            self.pio_host_turn();
        }
    }

    /// Runs up to `max_steps` steps; returns how many completed.
    pub fn run(&mut self, max_steps: u64) -> u64 {
        let start = self.steps;
        while self.steps - start < max_steps && self.stop.is_none() {
            self.step();
        }
        self.steps - start
    }

    /// `SimulationClock.tick`: fires every alarm due by the target time, each
    /// at its own time, then lands on the target.
    pub fn clock_tick(&mut self, delta_nanos: f64) {
        let target = self.clock.nanos + delta_nanos;
        while let Some(&(id, at)) = self.clock.alarms.first() {
            if at > target {
                break;
            }
            self.clock.alarms.remove(0);
            self.clock.nanos = at;
            self.fire_alarm(id);
        }
        self.clock.nanos = target;
    }

    fn fire_alarm(&mut self, id: clock::AlarmId) {
        match id {
            clock::AlarmId::Systick => self.systick_alarm_fired(),
            clock::AlarmId::Timer(i) => self.timer_alarm_fired(i as usize),
            clock::AlarmId::Watchdog => self.watchdog_alarm_fired(),
            clock::AlarmId::UsbRead(i) => self.usb_read_alarm_fired(i as usize),
            clock::AlarmId::UsbWrite(i) => self.usb_write_alarm_fired(i as usize),
            clock::AlarmId::UsbReset => self.usb_reset_alarm_fired(),
            clock::AlarmId::UsbSof => self.usb_generate_sof(),
            clock::AlarmId::Dma(i) => self.dma_transfer_alarm_fired(i as usize),
            clock::AlarmId::Pwm(i) => self.pwm_alarm_fired(i as usize),
            clock::AlarmId::AdcSample => self.adc_sample_alarm_fired(),
            clock::AlarmId::AdcMulti => self.adc_multi_alarm_fired(),
        }
    }
}
