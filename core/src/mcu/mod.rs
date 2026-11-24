//! Microcontroller emulators ported from their JavaScript references (W7).
//! Compiled only with a cargo feature (`avr`, `rp2040`, `fpga`); see plan-1.0.0
//! section 4.

#[cfg(feature = "avr")]
pub mod avr;
#[cfg(feature = "rp2040")]
pub mod rp2040;
#[cfg(feature = "fpga")]
pub mod fpga;
