//! Test-only exports for the lock-step harness (`scripts/avr-lockstep.mjs`).
//! Compiled only with the `avr` feature, so the shipped module, built
//! without it, exports none of these. Not part of the C ABI contract in
//! core/README.md; the integration ABI comes with package AVR4. Same shape
//! as the RP2040 port's `rp_*` exports.

use super::harness::{state_record, Machine, ScheduledIrq, STATE_WORDS};
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
    machine(m).irqs.push(ScheduledIrq {
        at_cycle: at_cycle as u64,
        interrupt: Interrupt {
            address: address as u8,
            flag_register: flag_register as u16,
            flag_mask: flag_mask as u8,
            enable_register: enable_register as u16,
            enable_mask: enable_mask as u8,
            constant: constant != 0,
            inverse_flag: inverse_flag != 0,
        },
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
