//! GPIO ports, ported from avr8js 0.21.0 `peripherals/gpio.js` (MIT,
//! Copyright (c) Uri Shaked and contributors; see NOTICE): `AVRIOPort` for
//! the ATmega328P's ports B, C and D with pin state, `setPin`, the write
//! listener (recorded as [`PortEvent`]s), the timers' pin overrides, the
//! external interrupts INT0/INT1 on port D and the pin-change interrupts.
//! Every register hook the constructor installs is a `Cpu` method here.

use super::cpu::Cpu;
use super::interrupt::Interrupt;

/// `PinState`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PinState {
    Low = 0,
    High = 1,
    Input = 2,
    InputPullUp = 3,
}

/// `PinOverrideMode`: how a timer's compare unit takes over a pin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinOverride {
    None,
    Enable,
    Set,
    Clear,
    Toggle,
}

/// One call of a port's write listener: `(newValue, prevValue)` plus what
/// `ArduinoMcu`'s listener reads at that moment: `cpu.cycles` and
/// `port.pinState(bit)` for the eight pins.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortEvent {
    pub port: u8,
    pub cycle: u64,
    pub value: u8,
    pub prev: u8,
    pub states: [PinState; 8],
}

#[derive(Clone, Copy)]
pub struct ExtConfig {
    pub eicr: u16,
    pub eimsk: u16,
    pub eifr: u16,
    pub index: u8,
    pub isc_offset: u8,
    pub interrupt: u8,
}

#[derive(Clone, Copy)]
pub struct PinChangeConfig {
    pub pcie: u8,
    pub pcicr: u16,
    pub pcifr: u16,
    pub pcmsk: u16,
    pub interrupt: u8,
    pub mask: u8,
    pub offset: u8,
}

pub struct PortConfig {
    pub pin: u16,
    pub ddr: u16,
    pub port: u16,
    pub pin_change: Option<PinChangeConfig>,
    pub external: [Option<ExtConfig>; 8],
}

const INT0: ExtConfig = ExtConfig { eicr: 0x69, eimsk: 0x3d, eifr: 0x3c, index: 0, isc_offset: 0, interrupt: 2 };
const INT1: ExtConfig = ExtConfig { eicr: 0x69, eimsk: 0x3d, eifr: 0x3c, index: 1, isc_offset: 2, interrupt: 4 };
const PCINT0: PinChangeConfig = PinChangeConfig { pcie: 0, pcicr: 0x68, pcifr: 0x3b, pcmsk: 0x6b, interrupt: 6, mask: 0xff, offset: 0 };
const PCINT1: PinChangeConfig = PinChangeConfig { pcie: 1, pcicr: 0x68, pcifr: 0x3b, pcmsk: 0x6c, interrupt: 8, mask: 0xff, offset: 0 };
const PCINT2: PinChangeConfig = PinChangeConfig { pcie: 2, pcicr: 0x68, pcifr: 0x3b, pcmsk: 0x6d, interrupt: 10, mask: 0xff, offset: 0 };

pub const PORT_B: usize = 0;
pub const PORT_C: usize = 1;
pub const PORT_D: usize = 2;

/// `portBConfig`, `portCConfig`, `portDConfig`.
pub fn port_config(i: usize) -> PortConfig {
    match i {
        PORT_B => PortConfig { pin: 0x23, ddr: 0x24, port: 0x25, pin_change: Some(PCINT0), external: [None; 8] },
        PORT_C => PortConfig { pin: 0x26, ddr: 0x27, port: 0x28, pin_change: Some(PCINT1), external: [None; 8] },
        _ => PortConfig {
            pin: 0x29,
            ddr: 0x2a,
            port: 0x2b,
            pin_change: Some(PCINT2),
            external: [None, None, Some(INT0), Some(INT1), None, None, None, None],
        },
    }
}

/// The port whose PORT register is at `addr` (`cpu.gpioByPort[addr]`).
pub fn port_by_port_address(addr: u16) -> Option<usize> {
    match addr {
        0x25 => Some(PORT_B),
        0x28 => Some(PORT_C),
        0x2b => Some(PORT_D),
        _ => None,
    }
}

/// Registers the ported peripherals model, whether or not they hook them
/// (the lock-step trap leaves these alone).
pub fn is_ported_register(a: u16) -> bool {
    matches!(a,
        0x23..=0x2b | 0x35..=0x37 | 0x3b..=0x3d | 0x44..=0x48 | 0x68 | 0x69 | 0x6b..=0x70
        | 0x78..=0x7c | 0x80..=0x82 | 0x84..=0x8b | 0xb0..=0xb4
        // EEPROM, SPI, TWI and USART0
        | 0x3f..=0x42 | 0x4c..=0x4e | 0xb8..=0xbd | 0xc0..=0xc2 | 0xc4..=0xc6)
}

pub struct Port {
    /// Constructed (`new AVRIOPort`); the others are placeholders.
    pub attached: bool,
    pub config: PortConfig,
    pub pin_value: u8,
    pub override_mask: u8,
    pub override_value: u8,
    pub last_value: u8,
    pub last_ddr: u8,
    pub last_pin: u8,
    pub open_collector: u8,
    /// The reference's `externalInts`: the interrupt objects, whose `constant`
    /// the level-trigger logic flips.
    pub ext_ints: [Option<Interrupt>; 8],
    pub pcint: Option<Interrupt>,
    /// `externalClockListeners`: the timer a pin clocks, if any.
    pub ext_clock: [Option<u8>; 8],
}

impl Port {
    pub fn new(i: usize) -> Port {
        let config = port_config(i);
        let mut ext_ints = [None; 8];
        for (pin, e) in config.external.iter().enumerate() {
            if let Some(e) = e {
                ext_ints[pin] = Some(Interrupt {
                    address: e.interrupt,
                    flag_register: e.eifr,
                    flag_mask: 1 << e.index,
                    enable_register: e.eimsk,
                    enable_mask: 1 << e.index,
                    constant: false,
                    inverse_flag: false,
                });
            }
        }
        let pcint = config.pin_change.map(|p| Interrupt {
            address: p.interrupt,
            flag_register: p.pcifr,
            flag_mask: 1 << p.pcie,
            enable_register: p.pcicr,
            enable_mask: 1 << p.pcie,
            constant: false,
            inverse_flag: false,
        });
        Port {
            attached: false,
            config,
            pin_value: 0,
            override_mask: 0xff,
            override_value: 0,
            last_value: 0,
            last_ddr: 0,
            last_pin: 0,
            open_collector: 0,
            ext_ints,
            pcint,
            ext_clock: [None; 8],
        }
    }
}

/// Which attachInterruptHook flavour a register is.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ExtHook {
    Other,
    Mask,
    Flag,
}

impl Cpu {
    /// `pinState(index)`.
    pub fn pin_state(&self, p: usize, index: u8) -> PinState {
        let port = &self.p.ports[p];
        let ddr = self.data[port.config.ddr as usize];
        let reg = self.data[port.config.port as usize];
        let bit = 1u8 << index;
        let open = if reg & bit != 0 { PinState::InputPullUp } else { PinState::Input };
        let high = if port.open_collector & bit != 0 { open } else { PinState::High };
        if ddr & bit != 0 {
            if port.last_value & bit != 0 { high } else { PinState::Low }
        } else {
            open
        }
    }

    /// `setPin(index, value)`: the level an outside source drives on the pin.
    pub fn set_pin(&mut self, p: usize, index: u8, value: bool) {
        let bit = 1u8 << index;
        let port = &mut self.p.ports[p];
        port.pin_value &= !bit;
        if value {
            port.pin_value |= bit;
        }
        let ddr = self.data[self.p.ports[p].config.ddr as usize];
        self.port_update_pin_register(p, ddr);
    }

    /// `timerOverridePin(pin, mode)`.
    pub fn timer_override_pin(&mut self, p: usize, pin: u8, mode: PinOverride) {
        if self.p.log_overrides {
            self.p.overrides.push((p, pin, mode));
        }
        let pin_mask = 1u8 << pin;
        let port_reg = self.data[self.p.ports[p].config.port as usize];
        let port = &mut self.p.ports[p];
        if mode == PinOverride::None {
            port.override_mask |= pin_mask;
            port.override_value &= !pin_mask;
        } else {
            port.override_mask &= !pin_mask;
            match mode {
                PinOverride::Enable => {
                    port.override_value &= !pin_mask;
                    port.override_value |= port_reg & pin_mask;
                }
                PinOverride::Set => port.override_value |= pin_mask,
                PinOverride::Clear => port.override_value &= !pin_mask,
                PinOverride::Toggle => port.override_value ^= pin_mask,
                PinOverride::None => {}
            }
        }
        let ddr = self.data[self.p.ports[p].config.ddr as usize];
        self.port_write_gpio(p, port_reg, ddr);
        self.port_update_pin_register(p, ddr);
    }

    /// `updatePinRegister(ddr)`. `lastPin` is stored after the per-pin loop,
    /// as in the reference, so a nested call from a pin's listener sees the
    /// old value.
    pub fn port_update_pin_register(&mut self, p: usize, ddr: u8) {
        let (new_pin, pin_reg) = {
            let port = &self.p.ports[p];
            ((port.pin_value & !ddr) | (port.last_value & ddr), port.config.pin as usize)
        };
        self.data[pin_reg] = new_pin;
        if self.p.ports[p].last_pin != new_pin {
            for index in 0..8u8 {
                let last = self.p.ports[p].last_pin;
                if (new_pin & (1 << index)) != (last & (1 << index)) {
                    let value = new_pin & (1 << index) != 0;
                    self.port_toggle_interrupt(p, index, value);
                    if let Some(t) = self.p.ports[p].ext_clock[index as usize] {
                        self.timer_external_clock(t as usize, value);
                    }
                }
            }
            self.p.ports[p].last_pin = new_pin;
        }
    }

    fn port_toggle_interrupt(&mut self, p: usize, pin: u8, rising: bool) {
        let cfg = self.p.ports[p].config.external[pin as usize];
        if let (Some(cfg), Some(mut ext)) = (cfg, self.p.ports[p].ext_ints[pin as usize]) {
            if self.data[cfg.eimsk as usize] & (1 << cfg.index) != 0 {
                let configuration = (self.data[cfg.eicr as usize] >> cfg.isc_offset) & 3;
                let generate;
                ext.constant = false;
                match configuration {
                    0 => {
                        generate = !rising;
                        ext.constant = true;
                    }
                    1 => generate = true,
                    2 => generate = !rising,
                    _ => generate = rising,
                }
                self.p.ports[p].ext_ints[pin as usize] = Some(ext);
                if generate {
                    self.set_interrupt_flag(ext);
                } else if ext.constant {
                    self.clear_interrupt(ext, true);
                }
            }
        }
        let pc = self.p.ports[p].config.pin_change;
        let pcint = self.p.ports[p].pcint;
        if let (Some(pc), Some(pcint)) = (pc, pcint) {
            if pc.mask & (1 << pin) != 0 && self.data[pc.pcmsk as usize] & (1 << (pin + pc.offset)) != 0 {
                self.set_interrupt_flag(pcint);
            }
        }
    }

    /// `checkExternalInterrupts()`: a low level on an enabled level-triggered
    /// pin queues its interrupt as a constant one.
    pub fn port_check_external_interrupts(&mut self, p: usize) {
        for pin in 0..8u8 {
            let Some(cfg) = self.p.ports[p].config.external[pin as usize] else { continue };
            let pin_value = self.p.ports[p].last_pin & (1 << pin) != 0;
            if self.data[cfg.eimsk as usize] & (1 << cfg.index) == 0 || pin_value {
                continue;
            }
            let configuration = (self.data[cfg.eicr as usize] >> cfg.isc_offset) & 3;
            if configuration == 0 {
                self.queue_interrupt(Interrupt {
                    address: cfg.interrupt,
                    flag_register: cfg.eifr,
                    flag_mask: 1 << cfg.index,
                    enable_register: cfg.eimsk,
                    enable_mask: 1 << cfg.index,
                    constant: true,
                    inverse_flag: false,
                });
            }
        }
    }

    /// `writeGpio(value, ddr)`: recomputes the driven level and, if it or the
    /// direction changed, calls the listener.
    pub fn port_write_gpio(&mut self, p: usize, value: u8, ddr: u8) {
        let port = &mut self.p.ports[p];
        let new_value = (((value & port.override_mask) | port.override_value) & ddr) | (value & !ddr);
        let prev = port.last_value;
        if new_value != prev || ddr != port.last_ddr {
            port.last_value = new_value;
            port.last_ddr = ddr;
            let mut states = [PinState::Low; 8];
            for (i, s) in states.iter_mut().enumerate() {
                *s = self.pin_state(p, i as u8);
            }
            let cycle = self.cycles;
            if self.p.listening {
                self.p.events.push(PortEvent { port: p as u8, cycle, value: new_value, prev, states });
            }
            // ArduinoMcu's own listener, installed before any other: parts
            // on the board's pins hear their drive changes.
            self.pin_targets_after_write(p);
        }
    }

    // ---- the register hooks the constructor installs -----------------------

    pub(crate) fn hook_port_ddr(&mut self, p: usize, value: u8) {
        let cfg = &self.p.ports[p].config;
        let (port_reg, ddr_reg) = (cfg.port as usize, cfg.ddr as usize);
        let port_value = self.data[port_reg];
        self.data[ddr_reg] = value;
        self.port_write_gpio(p, port_value, value);
        self.port_update_pin_register(p, value);
    }

    pub(crate) fn hook_port_port(&mut self, p: usize, value: u8) {
        let cfg = &self.p.ports[p].config;
        let (port_reg, ddr_reg) = (cfg.port as usize, cfg.ddr as usize);
        let ddr_mask = self.data[ddr_reg];
        self.data[port_reg] = value;
        self.port_write_gpio(p, value, ddr_mask);
        self.port_update_pin_register(p, ddr_mask);
    }

    pub(crate) fn hook_port_pin(&mut self, p: usize, value: u8, mask: u8) {
        let cfg = &self.p.ports[p].config;
        let (port_reg, ddr_reg) = (cfg.port as usize, cfg.ddr as usize);
        let old = self.data[port_reg];
        let ddr_mask = self.data[ddr_reg];
        let port_value = old ^ (value & mask);
        self.data[port_reg] = port_value;
        self.port_write_gpio(p, port_value, ddr_mask);
        self.port_update_pin_register(p, ddr_mask);
    }

    /// `attachInterruptHook(register, type)`'s closure.
    pub(crate) fn hook_external(&mut self, register: usize, kind: ExtHook, value: u8) {
        if kind != ExtHook::Flag {
            self.data[register] = value;
        }
        for g in 0..self.p.ports.len() {
            if !self.p.ports[g].attached {
                continue;
            }
            for pin in 0..8 {
                let Some(ext) = self.p.ports[g].ext_ints[pin] else { continue };
                if kind == ExtHook::Mask {
                    self.update_interrupt_enable(ext, value);
                }
                if !ext.constant && kind == ExtHook::Flag {
                    self.clear_interrupt_by_flag(ext, value);
                }
            }
            self.port_check_external_interrupts(g);
        }
    }

    pub(crate) fn hook_pcifr(&mut self, value: u8) {
        for g in 0..self.p.ports.len() {
            if !self.p.ports[g].attached {
                continue;
            }
            if let Some(pcint) = self.p.ports[g].pcint {
                self.clear_interrupt_by_flag(pcint, value);
            }
        }
    }

    pub(crate) fn hook_pcmsk(&mut self, register: usize, value: u8) {
        self.data[register] = value;
        for g in 0..self.p.ports.len() {
            if !self.p.ports[g].attached {
                continue;
            }
            if let Some(pcint) = self.p.ports[g].pcint {
                self.update_interrupt_enable(pcint, value);
            }
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for PinState {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PinState::Low => 0,
            PinState::High => 0,
            PinState::Input => 0,
            PinState::InputPullUp => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for PinOverride {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            PinOverride::None => 0,
            PinOverride::Enable => 0,
            PinOverride::Set => 0,
            PinOverride::Clear => 0,
            PinOverride::Toggle => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for PortEvent {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.port, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cycle, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.value, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.prev, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.states, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ExtConfig {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.eicr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.eimsk, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.eifr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.isc_offset, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interrupt, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PinChangeConfig {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pcie, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pcicr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pcifr, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pcmsk, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.interrupt, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mask, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.offset, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for ExtHook {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            ExtHook::Other => 0,
            ExtHook::Mask => 0,
            ExtHook::Flag => 0,
        }
    }
}
