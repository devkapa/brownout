//! Test-only exports for the lock-step harness (`scripts/avr-lockstep.mjs`).
//! Compiled only with the `avr` feature, so the shipped module, built
//! without it, exports none of these. Not part of the C ABI contract in
//! core/README.md; the integration ABI comes with package AVR4. Same shape
//! as the RP2040 port's `rp_*` exports.

use super::harness::{state_record, Input, Machine, ScheduledInput, STATE_WORDS};
use super::instr::decode;
use super::interrupt::Interrupt;

#[no_mangle]
pub extern "C" fn avr_alloc(len: u32) -> *mut u8 {
    let buf: Box<[u8]> = vec![0u8; len as usize].into_boxed_slice();
    Box::into_raw(buf) as *mut u8
}

/// # Safety
/// `ptr` and `len` must come from one `avr_alloc` call.
#[no_mangle]
pub unsafe extern "C" fn avr_free(ptr: *mut u8, len: u32) {
    if !ptr.is_null() {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len as usize)));
    }
}

/// `new CPU(flashWords)` from `len` bytes of little-endian program words.
///
/// # Safety
/// `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn avr_create(ptr: *const u8, len: u32) -> *mut Machine {
    let bytes = std::slice::from_raw_parts(ptr, len as usize);
    let prog: Vec<u16> = bytes.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
    Box::into_raw(Box::new(Machine::new(prog)))
}

/// # Safety
/// `m` must come from `avr_create` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn avr_destroy(m: *mut Machine) {
    if !m.is_null() {
        drop(Box::from_raw(m));
    }
}

unsafe fn machine<'a>(m: *mut Machine) -> &'a mut Machine {
    &mut *m
}

/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_set_io_trap(m: *mut Machine, on: u32) {
    machine(m).cpu.io_trap = on != 0;
}

/// Schedules `setInterruptFlag(interrupt)` before the first instruction at or
/// after `at_cycle`.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn avr_schedule_irq(
    m: *mut Machine,
    at_cycle: f64,
    address: u32,
    flag_register: u32,
    flag_mask: u32,
    enable_register: u32,
    enable_mask: u32,
    constant: u32,
    inverse_flag: u32,
) {
    machine(m).inputs.push(ScheduledInput {
        at_cycle: at_cycle as u64,
        input: Input::Irq(Interrupt {
            address: address as u8,
            flag_register: flag_register as u16,
            flag_mask: flag_mask as u8,
            enable_register: enable_register as u16,
            enable_mask: enable_mask as u8,
            constant: constant != 0,
            inverse_flag: inverse_flag != 0,
        }),
    });
}

/// Runs up to `max_steps` instructions; returns how many ran.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_run(m: *mut Machine, max_steps: f64) -> f64 {
    machine(m).run(max_steps as u64) as f64
}

/// Writes the compared state record (`STATE_WORDS` f64s, layout in
/// `harness::state_record`).
///
/// # Safety
/// `m` from `avr_create`; `out` valid for `STATE_WORDS` f64s.
#[no_mangle]
pub unsafe extern "C" fn avr_state(m: *mut Machine, out: *mut f64) {
    state_record(&machine(m).cpu, std::slice::from_raw_parts_mut(out, STATE_WORDS));
}

#[no_mangle]
pub extern "C" fn avr_state_words() -> u32 {
    STATE_WORDS as u32
}

/// Pointer to the data space (registers, I/O, SRAM) and its length via
/// `avr_data_len`, for byte-level diffs on a mismatch.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_data_ptr(m: *mut Machine) -> *const u8 {
    machine(m).cpu.data.as_ptr()
}

/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_data_len(m: *mut Machine) -> u32 {
    machine(m).cpu.data.len() as u32
}

/// The mnemonic `decode` gives an opcode word, written to `out` (up to
/// `cap` bytes); returns its length.
///
/// # Safety
/// `out` valid for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn avr_decode_name(opcode: u32, out: *mut u8, cap: u32) -> u32 {
    let name = decode(opcode as u16).name();
    let n = name.len().min(cap as usize);
    std::ptr::copy_nonoverlapping(name.as_ptr(), out, n);
    n as u32
}

/// Constructs ports B, C, D, timers 0-2 and the ADC, as `new ArduinoMcu` does.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_attach(m: *mut Machine) {
    machine(m).cpu.attach_arduino_peripherals();
}

/// Whether `addr` is a register the ported peripherals model.
#[no_mangle]
pub extern "C" fn avr_is_ported(addr: u32) -> u32 {
    super::gpio::is_ported_register(addr as u16) as u32
}

/// Schedules `port.setPin(pin, level)` (port 0 B, 1 C, 2 D).
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_schedule_pin(m: *mut Machine, at_cycle: f64, port: u32, pin: u32, level: u32) {
    machine(m).inputs.push(ScheduledInput {
        at_cycle: at_cycle as u64,
        input: Input::Pin { port: port as u8, pin: pin as u8, level: level != 0 },
    });
}

/// Schedules `adc.channelValues[channel] = volts`.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_schedule_adc(m: *mut Machine, at_cycle: f64, channel: u32, volts: f64) {
    machine(m).inputs.push(ScheduledInput { at_cycle: at_cycle as u64, input: Input::Adc { channel: channel as u8, volts } });
}

/// Moves the GPIO write-listener events recorded so far into `out` (up to
/// `cap` records of 5 f64s: port, cycle, value, prev, the eight pin states
/// packed two bits each) and returns how many it wrote.
///
/// # Safety
/// `m` from `avr_create`; `out` valid for `cap * 5` f64s.
#[no_mangle]
pub unsafe extern "C" fn avr_drain_events(m: *mut Machine, out: *mut f64, cap: u32) -> u32 {
    let ev = &mut machine(m).cpu.p.events;
    let n = ev.len().min(cap as usize);
    let buf = std::slice::from_raw_parts_mut(out, n * 5);
    for (i, e) in ev.drain(..n).enumerate() {
        let mut packed = 0u32;
        for (b, s) in e.states.iter().enumerate() {
            packed |= (*s as u32) << (2 * b);
        }
        buf[i * 5] = e.port as f64;
        buf[i * 5 + 1] = e.cycle as f64;
        buf[i * 5 + 2] = e.value as f64;
        buf[i * 5 + 3] = e.prev as f64;
        buf[i * 5 + 4] = packed as f64;
    }
    n as u32
}

/// Schedules `cpu.writeData(addr, value)`.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_schedule_write(m: *mut Machine, at_cycle: f64, addr: u32, value: u32) {
    machine(m).inputs.push(ScheduledInput { at_cycle: at_cycle as u64, input: Input::Write { addr: addr as u16, value: value as u8 } });
}
