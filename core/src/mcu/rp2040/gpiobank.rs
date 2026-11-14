//! The IO bank (per-pin STATUS and CTRL, the interrupt registers) and the
//! pad banks (per-pin pad control), ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/io.ts` and `src/peripherals/pads.ts`. The pin state they
//! read and write is `GPIOPin` (`src/gpio-pin.ts`), which lives in `sio.rs`
//! beside the SIO fast path that drives it.
//!
//! IO_QSPI is an `UnimplementedPeripheral` in the reference, so the QSPI
//! pins' CTRL and interrupt registers stay at their reset values; only the
//! QSPI pads reach them.

use super::sio::{Pin, FUNCTION_SIO, NUM_GPIO, NUM_QSPI};
use super::Rp2040;

const GPIO_CTRL_LAST: u32 = 0x0ec;
const INTR0: u32 = 0xf0;
const PROC0_INTE0: u32 = 0x100;
const PROC0_INTF0: u32 = 0x110;
const PROC0_INTS0: u32 = 0x120;
const PROC0_INTS3: u32 = 0x12c;

const PADS_VOLTAGE_SELECT: u32 = 0;
const GPIO_FIRST: u32 = 0x4;
const GPIO_LAST: u32 = 0x78;
const QSPI_FIRST: u32 = 0x4;
const QSPI_LAST: u32 = 0x18;

const IRQ_EDGE_HIGH: u32 = 1 << 3;
const IRQ_EDGE_LOW: u32 = 1 << 2;
const IRQ_LEVEL_HIGH: u32 = 1 << 1;
const IRQ_LEVEL_LOW: u32 = 1 << 0;
/// The IO_BANK0 interrupt line (`IRQ.IO_BANK0`).
const IRQ_IO_BANK0: u32 = 13;

/// The two pad banks' VOLTAGE_SELECT registers (the only pad state that is
/// not a pin's).
pub struct GpioBank {
    pub voltage_select: [u32; 2],
}

impl Default for GpioBank {
    fn default() -> Self {
        Self::new()
    }
}

impl GpioBank {
    pub fn new() -> Self {
        GpioBank { voltage_select: [0; 2] }
    }
}

fn apply_override(value: bool, override_type: u32) -> bool {
    match override_type {
        0 => value,
        1 => !value,
        2 => false,
        _ => true,
    }
}

fn raw_interrupt(p: &Pin) -> bool {
    (p.irq_status & p.irq_enable_mask) | p.irq_force_mask != 0
}

fn irq_value(p: &Pin) -> bool {
    apply_override(raw_interrupt(p), (p.ctrl >> 28) & 0x3)
}

fn input_enable(p: &Pin) -> bool {
    p.pad_value & 0x40 != 0
}

fn input_value(p: &Pin) -> bool {
    apply_override(p.raw_input_value && input_enable(p), (p.ctrl >> 16) & 0x3)
}

impl Rp2040 {
    /// `RP2040.updateIOInterrupt`.
    fn io_update_interrupt(&mut self) {
        let any = self.sio.gpio.iter().any(irq_value);
        self.set_interrupt(IRQ_IO_BANK0, any);
    }

    /// `GPIOPin.status` (datasheet 2.19.6). Only the SIO function drives or
    /// enables a pin here: PWM and PIO sit at their reset state until their
    /// packages arrive, where the reference reads them as zero.
    fn pin_status(&self, p: &Pin) -> u32 {
        let bit = 1u32 << (p.index & 31);
        let sio = p.ctrl & 0x1f == FUNCTION_SIO;
        let raw_oe = sio && self.sio.gpio_output_enable & bit != 0;
        let raw_out = sio && self.sio.gpio_value & bit != 0;
        let oe = apply_override(raw_oe, (p.ctrl >> 12) & 0x3);
        let out = apply_override(raw_out, (p.ctrl >> 8) & 0x3);
        (if irq_value(p) { 1 << 26 } else { 0 })
            | (if raw_interrupt(p) { 1 << 24 } else { 0 })
            | (if input_value(p) { 1 << 19 } else { 0 })
            | (if p.raw_input_value { 1 << 17 } else { 0 })
            | (if oe { 1 << 13 } else { 0 })
            | (if raw_oe { 1 << 12 } else { 0 })
            | (if out { 1 << 9 } else { 0 })
            | (if raw_out { 1 << 8 } else { 0 })
    }

    pub(crate) fn io_read(&mut self, offset: u32) -> u32 {
        if offset <= GPIO_CTRL_LAST {
            let index = (offset >> 3) as usize;
            let p = self.sio.gpio[index];
            return if offset & 0x4 != 0 { p.ctrl } else { self.pin_status(&p) };
        }
        if (INTR0..=PROC0_INTS3).contains(&offset) {
            let start = ((offset & 0xf) * 2) as usize;
            let register = offset & !0xf;
            let mut result = 0u32;
            for index in (0..8usize).rev() {
                let Some(pin) = self.sio.gpio.get(index + start) else { continue };
                result <<= 4;
                match register {
                    INTR0 => result |= pin.irq_status,
                    PROC0_INTE0 => result |= pin.irq_enable_mask,
                    PROC0_INTF0 => result |= pin.irq_force_mask,
                    PROC0_INTS0 => result |= (pin.irq_status & pin.irq_enable_mask) | pin.irq_force_mask,
                    _ => {}
                }
            }
            return result;
        }
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
        0xffff_ffff
    }

    pub(crate) fn io_write(&mut self, offset: u32, value: u32, raw: u32) {
        if offset <= GPIO_CTRL_LAST {
            if offset & 0x4 != 0 {
                let index = (offset >> 3) as usize;
                self.sio.gpio[index].ctrl = value;
                self.gpio_check_for_updates(index);
            }
            return;
        }
        if (INTR0..=PROC0_INTS3).contains(&offset) {
            let start = ((offset & 0xf) * 2) as usize;
            let register = offset & !0xf;
            for index in 0..8usize {
                let i = index + start;
                if i >= NUM_GPIO {
                    continue;
                }
                let pin_value = (value >> (index * 4)) & 0xf;
                let pin_raw = (raw >> (index * 4)) & 0xf;
                match register {
                    INTR0 => {
                        // `GPIOPin.updateIRQValue`: edge flags clear on a 1.
                        let p = &mut self.sio.gpio[i];
                        if pin_raw & IRQ_EDGE_LOW != 0 && p.irq_status & IRQ_EDGE_LOW != 0 {
                            p.irq_status &= !IRQ_EDGE_LOW;
                            self.io_update_interrupt();
                        }
                        let p = &mut self.sio.gpio[i];
                        if pin_raw & IRQ_EDGE_HIGH != 0 && p.irq_status & IRQ_EDGE_HIGH != 0 {
                            p.irq_status &= !IRQ_EDGE_HIGH;
                            self.io_update_interrupt();
                        }
                    }
                    PROC0_INTE0 => {
                        if self.sio.gpio[i].irq_enable_mask != pin_value {
                            self.sio.gpio[i].irq_enable_mask = pin_value;
                            self.io_update_interrupt();
                        }
                    }
                    PROC0_INTF0 => {
                        if self.sio.gpio[i].irq_force_mask != pin_value {
                            self.sio.gpio[i].irq_force_mask = pin_value;
                            self.io_update_interrupt();
                        }
                    }
                    _ => {}
                }
            }
            return;
        }
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
    }

    /// `GPIOPin.setInputValue` for a QSPI pin. Its interrupt masks are never
    /// writable (IO_QSPI is a stub), so the pin's IRQ output stays low and
    /// the reference's `updateIOInterrupt` call cannot fire.
    fn qspi_set_input_value(&mut self, index: usize, value: bool) {
        let p = &mut self.sio.qspi[index];
        p.raw_input_value = value;
        if value && input_enable(p) {
            p.irq_status |= IRQ_EDGE_HIGH | IRQ_LEVEL_HIGH;
            p.irq_status &= !IRQ_LEVEL_LOW;
        } else {
            p.irq_status |= IRQ_EDGE_LOW | IRQ_LEVEL_LOW;
            p.irq_status &= !IRQ_LEVEL_HIGH;
        }
    }

    pub(crate) fn pads_read(&mut self, qspi: bool, offset: u32) -> u32 {
        let (first, last) = if qspi { (QSPI_FIRST, QSPI_LAST) } else { (GPIO_FIRST, GPIO_LAST) };
        if offset >= first && offset <= last {
            let index = ((offset - first) >> 2) as usize;
            return if qspi { self.sio.qspi[index].pad_value } else { self.sio.gpio[index].pad_value };
        }
        if offset == PADS_VOLTAGE_SELECT {
            return self.periph.io.voltage_select[qspi as usize];
        }
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
        0xffff_ffff
    }

    pub(crate) fn pads_write(&mut self, qspi: bool, offset: u32, value: u32) {
        let (first, last) = if qspi { (QSPI_FIRST, QSPI_LAST) } else { (GPIO_FIRST, GPIO_LAST) };
        if offset >= first && offset <= last {
            let index = ((offset - first) >> 2) as usize;
            if qspi {
                debug_assert!(index < NUM_QSPI);
                let old_ie = input_enable(&self.sio.qspi[index]);
                self.sio.qspi[index].pad_value = value;
                // `checkForUpdates`: nothing listens to a QSPI pin.
                let state = self.sio.pin_value(&self.sio.qspi[index]);
                self.sio.qspi[index].last_value = state;
                if old_ie != input_enable(&self.sio.qspi[index]) {
                    let raw = self.sio.qspi[index].raw_input_value;
                    self.qspi_set_input_value(index, raw);
                }
            } else {
                let old_ie = input_enable(&self.sio.gpio[index]);
                self.sio.gpio[index].pad_value = value;
                self.gpio_check_for_updates(index);
                if old_ie != input_enable(&self.sio.gpio[index]) {
                    let raw = self.sio.gpio[index].raw_input_value;
                    self.set_input_value(index as u32, raw);
                }
            }
            return;
        }
        if offset == PADS_VOLTAGE_SELECT {
            self.periph.io.voltage_select[qspi as usize] = value & 1;
            return;
        }
        self.periph.unimplemented_accesses += 1;
        self.warnings += 1;
    }
}
