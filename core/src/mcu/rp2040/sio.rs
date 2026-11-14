//! SIO: the GPIO fast path, the hardware divider, the spinlocks and the two
//! interpolators, ported from rp2040js 1.3.3 (MIT): `src/sio.ts`. The pin
//! state SIO reads and drives is the part of `src/gpio-pin.ts` R1 needs; the
//! IO bank and pads that configure it are R6, so in R1 every pin keeps its
//! reset configuration.

use super::interp::Interpolator;
use super::{PinEvent, Rp2040};

const CPUID: u32 = 0x000;
const GPIO_IN: u32 = 0x004;
const GPIO_HI_IN: u32 = 0x008;
const GPIO_OUT: u32 = 0x010;
const GPIO_OUT_SET: u32 = 0x014;
const GPIO_OUT_CLR: u32 = 0x018;
const GPIO_OUT_XOR: u32 = 0x01c;
const GPIO_OE: u32 = 0x020;
const GPIO_OE_SET: u32 = 0x024;
const GPIO_OE_CLR: u32 = 0x028;
const GPIO_OE_XOR: u32 = 0x02c;
const GPIO_HI_OUT: u32 = 0x030;
const GPIO_HI_OUT_SET: u32 = 0x034;
const GPIO_HI_OUT_CLR: u32 = 0x038;
const GPIO_HI_OUT_XOR: u32 = 0x03c;
const GPIO_HI_OE: u32 = 0x040;
const GPIO_HI_OE_SET: u32 = 0x044;
const GPIO_HI_OE_CLR: u32 = 0x048;
const GPIO_HI_OE_XOR: u32 = 0x04c;
const GPIO_MASK: u32 = 0x3fff_ffff;
const DIV_UDIVIDEND: u32 = 0x060;
const DIV_UDIVISOR: u32 = 0x064;
const DIV_SDIVIDEND: u32 = 0x068;
const DIV_SDIVISOR: u32 = 0x06c;
const DIV_QUOTIENT: u32 = 0x070;
const DIV_REMAINDER: u32 = 0x074;
const DIV_CSR: u32 = 0x078;
const SPINLOCK_ST: u32 = 0x05c;
const SPINLOCK0: u32 = 0x100;
const SPINLOCK31: u32 = 0x17c;
/// INTERP0 occupies 0x080..0x0bf, INTERP1 the same layout at 0x0c0.
const INTERP0_BASE: u32 = 0x080;
const INTERP1_BASE: u32 = 0x0c0;

pub const NUM_GPIO: usize = 30;
pub const NUM_QSPI: usize = 6;

pub const FUNCTION_SIO: u32 = 5;

/// `GPIOPinState`.
pub const PIN_LOW: u32 = 0;
pub const PIN_HIGH: u32 = 1;
pub const PIN_INPUT: u32 = 2;
pub const PIN_INPUT_PULLUP: u32 = 3;
pub const PIN_INPUT_PULLDOWN: u32 = 4;
pub const PIN_INPUT_BUSKEEPER: u32 = 5;

const IRQ_EDGE_HIGH: u32 = 1 << 3;
const IRQ_EDGE_LOW: u32 = 1 << 2;
const IRQ_LEVEL_HIGH: u32 = 1 << 1;
const IRQ_LEVEL_LOW: u32 = 1 << 0;

/// The IO_BANK0 interrupt line.
const IRQ_IO_BANK0: u32 = 13;

fn apply_override(value: bool, override_type: u32) -> bool {
    match override_type {
        0 => value,
        1 => !value,
        2 => false,
        _ => true,
    }
}

/// One `GPIOPin`'s state at its reset configuration.
#[derive(Clone, Copy)]
pub struct Pin {
    pub index: u32,
    pub raw_input_value: bool,
    pub last_value: u32,
    pub ctrl: u32,
    pub pad_value: u32,
    pub irq_enable_mask: u32,
    pub irq_force_mask: u32,
    pub irq_status: u32,
}

impl Pin {
    fn new(index: u32) -> Self {
        Pin {
            index,
            raw_input_value: false,
            // Reference quirk: the constructor reads `this.value` before it
            // sets `ctrl` and `padValue`, so the remembered state is plain
            // Input, and a pin's first update reports Input -> its real
            // reset state (InputPullDown).
            last_value: PIN_INPUT,
            ctrl: 0x1f,
            pad_value: 0b0110110,
            irq_enable_mask: 0,
            irq_force_mask: 0,
            irq_status: 0,
        }
    }
    fn input_enable(&self) -> bool {
        self.pad_value & 0x40 != 0
    }
    fn function_select(&self) -> u32 {
        self.ctrl & 0x1f
    }
    fn input_value(&self) -> bool {
        apply_override(self.raw_input_value && self.input_enable(), (self.ctrl >> 16) & 0x3)
    }
    fn raw_interrupt(&self) -> bool {
        (self.irq_status & self.irq_enable_mask) | self.irq_force_mask != 0
    }
    fn irq_value(&self) -> bool {
        apply_override(self.raw_interrupt(), (self.ctrl >> 28) & 0x3)
    }
}

pub struct Sio {
    pub gpio_value: u32,
    pub gpio_output_enable: u32,
    pub qspi_gpio_value: u32,
    pub qspi_gpio_output_enable: u32,
    pub div_dividend: u32,
    pub div_divisor: u32,
    /// The reference keeps the quotient as a double (a signed or unsigned
    /// division, unrounded); every read truncates it to 32 bits, so the
    /// truncation is stored.
    pub div_quotient: u32,
    pub div_remainder: u32,
    pub div_csr: u32,
    pub spin_lock: u32,
    pub interp0: Interpolator,
    pub interp1: Interpolator,
    pub gpio: [Pin; NUM_GPIO],
    pub qspi: [Pin; NUM_QSPI],
}

impl Default for Sio {
    fn default() -> Self {
        Self::new()
    }
}

impl Sio {
    pub fn new() -> Self {
        let mut gpio = [Pin::new(0); NUM_GPIO];
        for (i, p) in gpio.iter_mut().enumerate() {
            *p = Pin::new(i as u32);
        }
        let mut qspi = [Pin::new(0); NUM_QSPI];
        for (i, p) in qspi.iter_mut().enumerate() {
            *p = Pin::new(i as u32);
        }
        Sio {
            gpio_value: 0,
            gpio_output_enable: 0,
            qspi_gpio_value: 0,
            qspi_gpio_output_enable: 0,
            div_dividend: 0,
            div_divisor: 1,
            div_quotient: 0,
            div_remainder: 0,
            div_csr: 0,
            spin_lock: 0,
            interp0: Interpolator::new(0),
            interp1: Interpolator::new(1),
            gpio,
            qspi,
        }
    }

    /// `GPIOPin.value`. Only the SIO function can drive a pin in R1 (PWM
    /// and PIO are R5/R6, and nothing in R1 can select them).
    pub fn pin_value(&self, pin: &Pin) -> u32 {
        let bit = 1u32 << (pin.index & 31);
        let sio = pin.function_select() == FUNCTION_SIO;
        let raw_oe = sio && self.gpio_output_enable & bit != 0;
        let raw_out = sio && self.gpio_value & bit != 0;
        let output_enable = apply_override(raw_oe, (pin.ctrl >> 12) & 0x3);
        if output_enable {
            if apply_override(raw_out, (pin.ctrl >> 8) & 0x3) {
                PIN_HIGH
            } else {
                PIN_LOW
            }
        } else {
            let pulldown = pin.pad_value & 4 != 0;
            let pullup = pin.pad_value & 8 != 0;
            if pulldown && pullup {
                PIN_INPUT_BUSKEEPER
            } else if pulldown {
                PIN_INPUT_PULLDOWN
            } else if pullup {
                PIN_INPUT_PULLUP
            } else {
                PIN_INPUT
            }
        }
    }

    /// `updateHardwareDivider`; the caller adds the 8 cycles.
    fn update_hardware_divider(&mut self, signed: bool) {
        if self.div_divisor == 0 {
            self.div_quotient = if self.div_dividend > 0 { 0xffff_ffff } else { 1 };
            self.div_remainder = self.div_dividend;
        } else if signed {
            let a = self.div_dividend as i32 as i64;
            let b = self.div_divisor as i32 as i64;
            // JavaScript's / then ToUint32 truncates toward zero, and % takes
            // the dividend's sign, as Rust's i64 / and % do.
            self.div_quotient = (a / b) as u32;
            self.div_remainder = (a % b) as u32;
        } else {
            self.div_quotient = self.div_dividend / self.div_divisor;
            self.div_remainder = self.div_dividend % self.div_divisor;
        }
        self.div_csr = 0b11;
    }
}

impl Rp2040 {
    fn divide(&mut self, signed: bool) {
        self.sio.update_hardware_divider(signed);
        self.cpu.cycles += 8;
    }

    pub fn sio_read(&mut self, offset: u32) -> u32 {
        let sio = &mut self.sio;
        if (SPINLOCK0..=SPINLOCK31).contains(&offset) {
            let bit = 1u32 << (((offset - SPINLOCK0) / 4) & 31);
            if sio.spin_lock & bit != 0 {
                return 0;
            }
            sio.spin_lock |= bit;
            return bit;
        }
        match offset {
            GPIO_IN => {
                let mut result = 0u32;
                for (i, p) in sio.gpio.iter().enumerate() {
                    if p.input_value() {
                        result |= 1 << i;
                    }
                }
                result
            }
            GPIO_HI_IN => {
                let mut result = 0u32;
                for (i, p) in sio.qspi.iter().enumerate() {
                    if p.input_value() {
                        result |= 1 << i;
                    }
                }
                result
            }
            GPIO_OUT => sio.gpio_value,
            GPIO_OE => sio.gpio_output_enable,
            GPIO_HI_OUT => sio.qspi_gpio_value,
            GPIO_HI_OE => sio.qspi_gpio_output_enable,
            GPIO_OUT_SET | GPIO_OUT_CLR | GPIO_OUT_XOR | GPIO_OE_SET | GPIO_OE_CLR | GPIO_OE_XOR | GPIO_HI_OUT_SET
            | GPIO_HI_OUT_CLR | GPIO_HI_OUT_XOR | GPIO_HI_OE_SET | GPIO_HI_OE_CLR | GPIO_HI_OE_XOR => 0,
            CPUID => 0,
            SPINLOCK_ST => sio.spin_lock,
            DIV_UDIVIDEND | DIV_SDIVIDEND => sio.div_dividend,
            DIV_UDIVISOR | DIV_SDIVISOR => sio.div_divisor,
            DIV_QUOTIENT => {
                sio.div_csr &= !0b10;
                sio.div_quotient
            }
            DIV_REMAINDER => sio.div_remainder,
            DIV_CSR => sio.div_csr,
            INTERP0_BASE..=0x0bf | INTERP1_BASE..=0x0ff => {
                let (interp, reg) = if offset < INTERP1_BASE {
                    (&mut sio.interp0, offset - INTERP0_BASE)
                } else {
                    (&mut sio.interp1, offset - INTERP1_BASE)
                };
                match interp.read(reg) {
                    Some(v) => v,
                    None => {
                        self.warnings += 1;
                        0xffff_ffff
                    }
                }
            }
            _ => {
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub fn sio_write(&mut self, offset: u32, value: u32) {
        if (SPINLOCK0..=SPINLOCK31).contains(&offset) {
            let bit = 1u32 << (((offset - SPINLOCK0) / 4) & 31);
            self.sio.spin_lock &= !bit;
            return;
        }
        let prev_value = self.sio.gpio_value;
        let prev_oe = self.sio.gpio_output_enable;
        let sio = &mut self.sio;
        match offset {
            GPIO_OUT => sio.gpio_value = value & GPIO_MASK,
            GPIO_OUT_SET => sio.gpio_value |= value & GPIO_MASK,
            GPIO_OUT_CLR => sio.gpio_value &= !value,
            GPIO_OUT_XOR => sio.gpio_value ^= value & GPIO_MASK,
            GPIO_OE => sio.gpio_output_enable = value & GPIO_MASK,
            GPIO_OE_SET => sio.gpio_output_enable |= value & GPIO_MASK,
            GPIO_OE_CLR => sio.gpio_output_enable &= !value,
            GPIO_OE_XOR => sio.gpio_output_enable ^= value & GPIO_MASK,
            GPIO_HI_OUT => sio.qspi_gpio_value = value & GPIO_MASK,
            GPIO_HI_OUT_SET => sio.qspi_gpio_value |= value & GPIO_MASK,
            GPIO_HI_OUT_CLR => sio.qspi_gpio_value &= !value,
            GPIO_HI_OUT_XOR => sio.qspi_gpio_value ^= value & GPIO_MASK,
            GPIO_HI_OE => sio.qspi_gpio_output_enable = value & GPIO_MASK,
            GPIO_HI_OE_SET => sio.qspi_gpio_output_enable |= value & GPIO_MASK,
            GPIO_HI_OE_CLR => sio.qspi_gpio_output_enable &= !value,
            GPIO_HI_OE_XOR => sio.qspi_gpio_output_enable ^= value & GPIO_MASK,
            DIV_UDIVIDEND => {
                sio.div_dividend = value;
                self.divide(false);
            }
            DIV_SDIVIDEND => {
                sio.div_dividend = value;
                self.divide(true);
            }
            DIV_UDIVISOR => {
                sio.div_divisor = value;
                self.divide(false);
            }
            DIV_SDIVISOR => {
                sio.div_divisor = value;
                self.divide(true);
            }
            DIV_QUOTIENT => {
                sio.div_quotient = value;
                sio.div_csr = 0b11;
            }
            DIV_REMAINDER => {
                sio.div_remainder = value;
                sio.div_csr = 0b11;
            }
            INTERP0_BASE..=0x0bf | INTERP1_BASE..=0x0ff => {
                let (interp, reg) = if offset < INTERP1_BASE {
                    (&mut sio.interp0, offset - INTERP0_BASE)
                } else {
                    (&mut sio.interp1, offset - INTERP1_BASE)
                };
                if !interp.write(reg, value) {
                    self.warnings += 1;
                }
            }
            _ => self.warnings += 1,
        }
        let pins_to_update = (self.sio.gpio_value ^ prev_value) | (self.sio.gpio_output_enable ^ prev_oe);
        if pins_to_update != 0 {
            for i in 0..NUM_GPIO {
                if pins_to_update & (1 << i) != 0 {
                    self.gpio_check_for_updates(i);
                }
            }
        }
    }

    /// `GPIOPin.checkForUpdates`: a changed drive state reaches the pin's
    /// listeners, which the harness records as pin events.
    pub(crate) fn gpio_check_for_updates(&mut self, i: usize) {
        let value = self.sio.pin_value(&self.sio.gpio[i]);
        if value != self.sio.gpio[i].last_value {
            self.sio.gpio[i].last_value = value;
            self.pin_events.push(PinEvent { pin: i as u32, state: value, nanos: self.clock.nanos });
        }
    }

    /// `GPIOPin.setInputValue`, for the GPIOs a host drives. (The PWM and PIO
    /// hooks it also calls cannot fire in R1: no pin can select PWM, and no
    /// state machine runs.)
    pub fn set_input_value(&mut self, pin: u32, value: bool) {
        let i = pin as usize;
        if i >= NUM_GPIO {
            return;
        }
        let p = &mut self.sio.gpio[i];
        p.raw_input_value = value;
        let prev_irq = p.irq_value();
        if value && p.input_enable() {
            p.irq_status |= IRQ_EDGE_HIGH | IRQ_LEVEL_HIGH;
            p.irq_status &= !IRQ_LEVEL_LOW;
        } else {
            p.irq_status |= IRQ_EDGE_LOW | IRQ_LEVEL_LOW;
            p.irq_status &= !IRQ_LEVEL_HIGH;
        }
        if p.irq_value() != prev_irq {
            self.update_io_interrupt();
        }
    }

    /// `RP2040.updateIOInterrupt`.
    fn update_io_interrupt(&mut self) {
        let any = self.sio.gpio.iter().any(|p| p.irq_value());
        self.set_interrupt(IRQ_IO_BANK0, any);
    }
}
