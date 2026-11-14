//! The private peripheral bus: NVIC, the system control registers and
//! SysTick, ported from rp2040js 1.3.3 (MIT): `src/peripherals/ppb.ts`.

use super::clock::{AlarmId, PeriodicAlarm, Timer32, TimerMode};
use super::core::MAX_HARDWARE_IRQ;
use super::Rp2040;

pub const CPUID: u32 = 0xd00;
pub const ICSR: u32 = 0xd04;
pub const VTOR: u32 = 0xd08;
pub const SHPR2: u32 = 0xd1c;
pub const SHPR3: u32 = 0xd20;
pub const SYST_CSR: u32 = 0x010;
pub const SYST_RVR: u32 = 0x014;
pub const SYST_CVR: u32 = 0x018;
pub const SYST_CALIB: u32 = 0x01c;
pub const NVIC_ISER: u32 = 0x100;
pub const NVIC_ICER: u32 = 0x180;
pub const NVIC_ISPR: u32 = 0x200;
pub const NVIC_ICPR: u32 = 0x280;
pub const NVIC_IPR0: u32 = 0x400;
pub const NVIC_IPR7: u32 = 0x41c;

const NMIPENDSET: u32 = 1 << 31;
const PENDSVSET: u32 = 1 << 28;
const PENDSVCLR: u32 = 1 << 27;
const PENDSTSET: u32 = 1 << 26;
const PENDSTCLR: u32 = 1 << 25;
const ISRPENDING: u32 = 1 << 22;
const VECTPENDING_SHIFT: u32 = 12;
const VECTACTIVE_MASK: u32 = 0x1ff;

/// `clkSys`: SysTick counts the 125 MHz system clock.
const CLK_SYS: f64 = 125_000_000.0;

pub struct Ppb {
    pub systick_count_flag: bool,
    pub systick_clk_source: bool,
    pub systick_int_enable: bool,
    pub systick_reload: u32,
    pub systick_timer: Timer32,
    pub systick_alarm: PeriodicAlarm,
}

impl Default for Ppb {
    fn default() -> Self {
        Self::new()
    }
}

impl Ppb {
    pub fn new() -> Self {
        Ppb {
            systick_count_flag: false,
            systick_clk_source: false,
            systick_int_enable: false,
            systick_reload: 0,
            systick_timer: Timer32::new(CLK_SYS),
            systick_alarm: PeriodicAlarm { target_value: 0.0, enabled: false },
        }
    }
}

impl Rp2040 {
    /// The `RPPPB` constructor after the field initialisers: the SysTick
    /// timer and alarm setup, then `reset()`.
    pub(crate) fn ppb_construct(&mut self) {
        let now = self.clock.nanos;
        self.ppb.systick_timer.set_top(0xff_ffff as f64, now);
        self.systick_timer_updated();
        if self.ppb.systick_timer.set_mode(TimerMode::Decrement, now) {
            self.systick_timer_updated();
        }
        // `systickAlarm.target = 0` equals the initial target: no-op.
        // `systickAlarm.enable = true`:
        self.ppb.systick_alarm.enabled = true;
        if self.ppb.systick_timer.enabled {
            self.systick_schedule();
        }
        // reset()
        self.ppb_write(SYST_CSR, 0);
        self.ppb_write(SYST_RVR, 0xff_ffff);
        self.ppb.systick_timer.set(0xff_ffff as f64, self.clock.nanos);
        self.systick_timer_updated();
    }

    /// `Timer32PeriodicAlarm.schedule` for SysTick.
    fn systick_schedule(&mut self) {
        let now = self.clock.nanos;
        if let Some(d) = self.ppb.systick_alarm.delay_nanos(&self.ppb.systick_timer, now) {
            self.clock.schedule(AlarmId::Systick, d);
        }
    }

    /// The timer's listener: the periodic alarm's `update`.
    fn systick_timer_updated(&mut self) {
        self.clock.cancel(AlarmId::Systick);
        if self.ppb.systick_alarm.enabled && self.ppb.systick_timer.enabled {
            self.systick_schedule();
        }
    }

    /// `handleAlarm`: the reload callback, then reschedule.
    pub(crate) fn systick_alarm_fired(&mut self) {
        self.ppb.systick_count_flag = true;
        if self.ppb.systick_int_enable {
            self.cpu.pending_systick = true;
            self.cpu.interrupts_updated = true;
        }
        let reload = self.ppb.systick_reload as f64;
        self.ppb.systick_timer.set(reload, self.clock.nanos);
        self.systick_timer_updated();
        if self.ppb.systick_alarm.enabled && self.ppb.systick_timer.enabled {
            self.systick_schedule();
        }
    }

    pub fn ppb_read(&mut self, offset: u32) -> u32 {
        let cpu = &self.cpu;
        match offset {
            CPUID => 0x410c_c601,
            ICSR => {
                let pending = cpu.pending_interrupts != 0 || cpu.pending_pendsv || cpu.pending_systick || cpu.pending_svcall;
                (if cpu.pending_nmi { NMIPENDSET } else { 0 })
                    | (if cpu.pending_pendsv { PENDSVSET } else { 0 })
                    | (if cpu.pending_systick { PENDSTSET } else { 0 })
                    | (if pending { ISRPENDING } else { 0 })
                    | (cpu.vect_pending() << VECTPENDING_SHIFT)
                    | (cpu.ipsr & VECTACTIVE_MASK)
            }
            VTOR => cpu.vtor,
            NVIC_ISPR | NVIC_ICPR => cpu.pending_interrupts,
            NVIC_ISER | NVIC_ICER => cpu.enabled_interrupts,
            NVIC_IPR0..=NVIC_IPR7 if offset & 3 == 0 => {
                let reg_index = (offset - NVIC_IPR0) >> 2;
                let mut result = 0u32;
                for byte_index in 0..4 {
                    let interrupt_number = reg_index * 4 + byte_index;
                    for priority in 0..4u32 {
                        if cpu.interrupt_priorities[priority as usize] & (1 << interrupt_number) != 0 {
                            result |= priority << (8 * byte_index + 6);
                        }
                    }
                }
                result
            }
            SHPR2 => cpu.shpr2,
            SHPR3 => cpu.shpr3,
            SYST_CSR => {
                let ppb = &mut self.ppb;
                let v = (if ppb.systick_count_flag { 1 << 16 } else { 0 })
                    | (if ppb.systick_clk_source { 1 << 2 } else { 0 })
                    | (if ppb.systick_int_enable { 1 << 1 } else { 0 })
                    | (if ppb.systick_timer.enabled { 1 } else { 0 });
                ppb.systick_count_flag = false;
                v
            }
            SYST_CVR => self.ppb.systick_timer.counter(self.clock.nanos),
            SYST_RVR => self.ppb.systick_reload,
            SYST_CALIB => 0x0000_270f,
            _ => {
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub fn ppb_write(&mut self, offset: u32, value: u32) {
        let hardware_interrupt_mask = (1u32 << MAX_HARDWARE_IRQ) - 1;
        match offset {
            ICSR => {
                let cpu = &mut self.cpu;
                if value & NMIPENDSET != 0 {
                    cpu.pending_nmi = true;
                    cpu.interrupts_updated = true;
                }
                if value & PENDSVSET != 0 {
                    cpu.pending_pendsv = true;
                    cpu.interrupts_updated = true;
                }
                if value & PENDSVCLR != 0 {
                    cpu.pending_pendsv = false;
                }
                if value & PENDSTSET != 0 {
                    cpu.pending_systick = true;
                    cpu.interrupts_updated = true;
                }
                if value & PENDSTCLR != 0 {
                    cpu.pending_systick = false;
                }
            }
            VTOR => self.cpu.vtor = value,
            NVIC_ISPR => {
                self.cpu.pending_interrupts |= value;
                self.cpu.interrupts_updated = true;
            }
            // Reference quirk: ICPR cannot clear IRQs 0..24 (the hardware
            // lines the peripherals drive).
            NVIC_ICPR => self.cpu.pending_interrupts &= !value | hardware_interrupt_mask,
            NVIC_ISER => {
                self.cpu.enabled_interrupts |= value;
                self.cpu.interrupts_updated = true;
            }
            NVIC_ICER => self.cpu.enabled_interrupts &= !value,
            NVIC_IPR0..=NVIC_IPR7 if offset & 3 == 0 => {
                let reg_index = (offset - NVIC_IPR0) >> 2;
                for byte_index in 0..4 {
                    let interrupt_number = reg_index * 4 + byte_index;
                    let new_priority = ((value >> (8 * byte_index + 6)) & 0x3) as usize;
                    for p in 0..4 {
                        self.cpu.interrupt_priorities[p] &= !(1u32 << interrupt_number);
                    }
                    self.cpu.interrupt_priorities[new_priority] |= 1u32 << interrupt_number;
                }
                self.cpu.interrupts_updated = true;
            }
            SHPR2 => self.cpu.shpr2 = value,
            SHPR3 => self.cpu.shpr3 = value,
            SYST_CSR => {
                self.ppb.systick_clk_source = value & (1 << 2) != 0;
                self.ppb.systick_int_enable = value & (1 << 1) != 0;
                if self.ppb.systick_timer.set_enable(value & 1 != 0, self.clock.nanos) {
                    self.systick_timer_updated();
                }
            }
            SYST_CVR => {
                self.ppb.systick_timer.set(0.0, self.clock.nanos);
                self.systick_timer_updated();
            }
            SYST_RVR => self.ppb.systick_reload = value,
            _ => self.warnings += 1,
        }
    }
}
