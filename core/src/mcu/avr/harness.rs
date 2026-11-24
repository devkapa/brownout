//! The machine the lock-step harness drives: the CPU plus the host inputs
//! the harness applies at fixed cycle counts (plan-1.0.0 section 4.3). AVR1
//! has no peripherals, so the only input is raising an interrupt flag, which
//! is how a timer or pin would reach the CPU; AVR2's ports, timers and ADC
//! extend this with pin levels and ADC values.

use super::cpu::{Cpu, DEFAULT_SRAM_BYTES};
use super::instr::avr_instruction;
use super::interrupt::Interrupt;

/// Words in the record `state_record` writes.
pub const STATE_WORDS: usize = 113;

/// What the host does to the machine: raise an interrupt flag (what a
/// peripheral AVR1 lacks would do), drive a pin, set an ADC channel's volts,
/// write a register or type a serial byte.
#[derive(Clone, Copy, Debug)]
pub enum Input {
    Irq(Interrupt),
    Pin { port: u8, pin: u8, level: bool },
    Adc { channel: u8, volts: f64 },
    /// A register write from the program's side (`cpu.writeData`).
    Write { addr: u16, value: u8 },
    /// A byte typed into the serial port (`ArduinoMcu.writeSerialInput`).
    SerialRx { byte: u8 },
}

/// An input applied before the first instruction that starts at or after
/// `at_cycle`, on both sides of the harness, in scheduled order.
#[derive(Clone, Copy, Debug)]
pub struct ScheduledInput {
    pub at_cycle: u64,
    pub input: Input,
}

pub struct Machine {
    pub cpu: Cpu,
    pub inputs: Vec<ScheduledInput>,
    pub steps: u64,
}

impl Machine {
    pub fn new(prog: Vec<u16>) -> Self {
        Machine { cpu: Cpu::new(prog, DEFAULT_SRAM_BYTES), inputs: Vec::new(), steps: 0 }
    }

    fn apply_due_inputs(&mut self) {
        // Due against the boundary's cycle count, read once: an input can move
        // the count itself (an EEPROM read halts the CPU 4 cycles), and the JS
        // side asks which inputs are due once per boundary.
        let now = self.cpu.cycles;
        let mut i = 0;
        while i < self.inputs.len() {
            if self.inputs[i].at_cycle <= now {
                let inp = self.inputs.remove(i);
                match inp.input {
                    Input::Irq(irq) => self.cpu.set_interrupt_flag(irq),
                    Input::Pin { port, pin, level } => {
                        // `ArduinoMcu.setInputBit` once pin targets are on the board.
                        if self.cpu.p.parts.pins.is_empty() {
                            self.cpu.set_pin(port as usize, pin, level)
                        } else {
                            self.cpu.set_input_bit(port, pin, level)
                        }
                    }
                    Input::Adc { channel, volts } => self.cpu.set_adc_channel(channel as usize, volts),
                    Input::Write { addr, value } => self.cpu.write_data(addr as i32, value),
                    Input::SerialRx { byte } => self.cpu.serial_write(&[byte]),
                }
            } else {
                i += 1;
            }
        }
    }

    /// One iteration of `ArduinoMcu.step`'s loop: `avrInstruction`, then
    /// `tick`.
    pub fn step(&mut self) {
        if !self.inputs.is_empty() {
            self.apply_due_inputs();
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
/// interrupt (-1 none), 5 first unported-register access address (-1 none),
/// 6 its write flag, 7 hash of the whole data space, 8 clock events pending,
/// 9 watchdog resets, 10..42 r0..r31, 42..46 the pending-interrupt bitmap,
/// 46 hash of the clock events' cycles, 47..77 the three timers (10 words
/// each: tcnt, lastCycle, ocrA, ocrB, icr, divider, flags, nextOcrA,
/// nextOcrB, tcntNext), 77..95 the three ports (pinValue, lastValue,
/// lastDdr, lastPin, overrideMask, overrideValue), 95 ADC converting,
/// 96 ADC conversionCycles, 97 GPIO events produced so far, 98 busy flags
/// (USART rxBusy 1, SPI transmissionActive 2, TWI busy 4), 99 USART rxByte,
/// 100 bytes transmitted, 101 FNV-1a over them, 102 typed bytes not yet
/// taken, 103 EEPROM writeEnabledCycles, 104 writeCompleteCycles, 105 hash of
/// the EEPROM, 106/107 the transmitter's shifting and pending bytes,
/// 108/109 the timed I2C steps and their hash, 110/111 the receiver's held
/// frame and overrun, 112 a hash of the board's parts (`BoardParts::state_hash`).
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
    for (i, p) in cpu.pending_interrupts.iter().enumerate() {
        if p.is_some() {
            out[42 + i / 32] += (1u64 << (i % 32)) as f64;
        }
    }
    let mut h: u32 = 0x811c9dc5;
    for e in &cpu.clock_events {
        h ^= e.cycles as u32;
        h = h.wrapping_mul(0x01000193);
    }
    out[46] = h as f64;
    for (t, tm) in cpu.p.timers.iter().enumerate().take(3) {
        let b = 47 + t * 10;
        out[b] = tm.tcnt as f64;
        out[b + 1] = tm.last_cycle as f64;
        out[b + 2] = tm.ocr_a as f64;
        out[b + 3] = tm.ocr_b as f64;
        out[b + 4] = tm.icr as f64;
        out[b + 5] = tm.divider as f64;
        out[b + 6] = (tm.counting_up as u8 | (tm.tcnt_updated as u8) << 1 | (tm.update_divider as u8) << 2) as f64;
        out[b + 7] = tm.next_ocr_a as f64;
        out[b + 8] = tm.next_ocr_b as f64;
        out[b + 9] = tm.tcnt_next as f64;
    }
    for (i, p) in cpu.p.ports.iter().enumerate().take(3) {
        let b = 77 + i * 6;
        out[b] = p.pin_value as f64;
        out[b + 1] = p.last_value as f64;
        out[b + 2] = p.last_ddr as f64;
        out[b + 3] = p.last_pin as f64;
        out[b + 4] = p.override_mask as f64;
        out[b + 5] = p.override_value as f64;
    }
    if let Some(a) = &cpu.p.adc {
        out[95] = a.converting as u8 as f64;
        out[96] = a.conversion_cycles as f64;
    }
    out[97] = cpu.p.events.len() as f64;
    let mut busy = 0u8;
    if let Some(u) = &cpu.p.usart {
        busy |= (u.rx_busy || u.rx_receiving) as u8;
        // The datasheet receiver's FIFO, packed: length, then each frame's
        // value and DOR flag (rx_byte is the reference's single buffer).
        let mut fifo = u.rx_fifo.len() as f64;
        for (i, (v, dor)) in u.rx_fifo.iter().enumerate() {
            fifo += (*v as f64 + 256.0 * *dor as u8 as f64) * 4f64 * 512f64.powi(i as i32);
        }
        out[99] = if u.datasheet { fifo } else { u.rx_byte as f64 };
        out[110] = u.rx_held as f64;
        out[111] = u.rx_overrun as u8 as f64;
        out[100] = u.tx_count as f64;
        out[101] = u.tx_hash as f64;
        out[102] = u.rx_queue.len() as f64;
        out[106] = u.tx_shifting as u8 as f64;
        out[107] = u.tx_pending as f64;
    }
    if let Some(s) = &cpu.p.spi {
        busy |= (s.transmission_active as u8) << 1;
    }
    if let Some(t) = &cpu.p.twi {
        busy |= (t.busy as u8) << 2;
        out[108] = t.timed_steps as f64;
        out[109] = t.timed_hash as f64;
    }
    out[98] = busy as f64;
    if cpu.p.attached {
        out[112] = cpu.p.parts.state_hash() as f64;
    }
    if let Some(e) = &cpu.p.eeprom {
        out[103] = e.write_enabled_cycles as f64;
        out[104] = e.write_complete_cycles as f64;
        out[105] = fnv1a(&e.memory) as f64;
    }
}
