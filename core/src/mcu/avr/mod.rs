//! ATmega328P emulator, ported from avr8js 0.21.0 (MIT, Copyright (c) Uri
//! Shaked and contributors; see NOTICE). Package AVR1 (plan-1.0.0 section
//! 4.2): the CPU (`cpu.rs`), every instruction (`instr.rs`) and interrupt
//! entry (`interrupt.rs`). Ports, timers and the ADC are AVR2; the USART, SPI, TWI
//! and EEPROM (`usart.rs`, `spi.rs`, `twi.rs`, `eeprom.rs`) joined them when
//! brownout's `ArduinoMcu` started constructing them (decision 10 (b)).
//!
//! The port is behaviour-exact against the reference, quirks included: the
//! lock-step harness (`scripts/avr-lockstep.mjs`) holds it to zero
//! divergence, so where avr8js departs from the AVR instruction set manual
//! the port departs the same way. `instr.rs` lists those places and the
//! unit tests name each one ("reference quirk").

pub mod abi;
pub mod adc;
pub mod bmp280;
pub mod bus;
pub mod cpu;
pub mod debug;
pub mod dht;
pub mod eeprom;
pub mod gpio;
pub mod harness;
pub mod instr;
pub mod interrupt;
pub mod parts;
pub mod periph;
pub mod spi;
pub mod ssd1306;
pub mod timer;
pub mod twi;
pub mod usart;

#[cfg(test)]
mod tests;
#[cfg(test)]
mod parts_tests;
#[cfg(test)]
mod periph_tests;
#[cfg(test)]
mod upstream_tests;
#[cfg(test)]
mod upstream_periph_tests;
#[cfg(test)]
mod upstream_comms_tests;

pub use cpu::{Cpu, IoHit};
pub use instr::{avr_instruction, decode, Op};
pub use interrupt::Interrupt;
