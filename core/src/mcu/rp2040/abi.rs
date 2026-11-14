//! Test-only exports for the lock-step harness (`scripts/rp2040-lockstep.mjs`).
//! Compiled only with the `rp2040` feature, so the shipped module, built
//! without it, exports none of these. Not part of the C ABI contract in
//! core/README.md; the integration ABI comes with package R7.

use super::{Rp2040, ScheduledInput, Stop};

/// Words in the state record `rp_state` writes, all as f64.
pub const STATE_WORDS: usize = 108;

#[no_mangle]
pub extern "C" fn rp_alloc(len: u32) -> *mut u8 {
    let buf: Box<[u8]> = vec![0u8; len as usize].into_boxed_slice();
    Box::into_raw(buf) as *mut u8
}

/// # Safety
/// `ptr` and `len` must come from one `rp_alloc` call.
#[no_mangle]
pub unsafe extern "C" fn rp_free(ptr: *mut u8, len: u32) {
    if !ptr.is_null() {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len as usize)));
    }
}

#[no_mangle]
pub extern "C" fn rp_create() -> *mut Rp2040 {
    Box::into_raw(Box::new(Rp2040::new()))
}

/// # Safety
/// `m` must come from `rp_create` and not be used afterwards.
#[no_mangle]
pub unsafe extern "C" fn rp_destroy(m: *mut Rp2040) {
    if !m.is_null() {
        drop(Box::from_raw(m));
    }
}

unsafe fn machine<'a>(m: *mut Rp2040) -> &'a mut Rp2040 {
    &mut *m
}

/// `loadBootrom`: `len` bytes of little-endian words, then `reset()`.
///
/// # Safety
/// `m` from `rp_create`; `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_load_bootrom(m: *mut Rp2040, ptr: *const u8, len: u32) {
    let bytes = std::slice::from_raw_parts(ptr, len as usize);
    let words: Vec<u32> = bytes.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect();
    machine(m).load_bootrom(&words);
}

/// Copies bytes into flash at `offset` (the UF2 loader's `flash.set`).
///
/// # Safety
/// `m` from `rp_create`; `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_flash_write(m: *mut Rp2040, offset: u32, ptr: *const u8, len: u32) -> i32 {
    let bytes = std::slice::from_raw_parts(ptr, len as usize);
    let mach = machine(m);
    let off = offset as usize;
    if off + bytes.len() > mach.flash.len() {
        return -1;
    }
    mach.flash[off..off + bytes.len()].copy_from_slice(bytes);
    mach.mark_flash_dirty(off, bytes.len());
    0
}

/// brownout's `loadUF2IntoFlash` over `len` bytes of a UF2 image; returns the
/// blocks applied.
///
/// # Safety
/// `m` from `rp_create`; `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_load_uf2(m: *mut Rp2040, ptr: *const u8, len: u32) -> u32 {
    machine(m).load_uf2(std::slice::from_raw_parts(ptr, len as usize))
}

/// FNV-1a over flash (16 MB): a checkpoint-rate comparison, not per step.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_flash_hash(m: *mut Rp2040) -> u32 {
    let mach = machine(m);
    mach.flash_dirty.clear();
    mach.flash_hash()
}

/// FNV-1a over the flash pages written since the last flash hash, then
/// forgets them.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_flash_dirty_hash(m: *mut Rp2040) -> u32 {
    machine(m).flash_dirty_hash()
}

/// Turns the wrapper's read-holding policy on or off (off is plain USBCDC).
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_cdc_hold_reads(m: *mut Rp2040, on: u32) {
    machine(m).periph.cdc.hold_reads = on != 0;
}

/// Sets the number of completed steps between the host's event-loop turns
/// (`Rp2040::pio_host_turn`); 0 leaves the turn to the caller.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_set_host_turn(m: *mut Rp2040, steps: f64) {
    machine(m).host_turn_steps = steps as u64;
}

/// Feeds REPL bytes to the CDC host at a fixed cycle count.
///
/// # Safety
/// `m` from `rp_create`; `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_schedule_repl(m: *mut Rp2040, at_cycle: f64, ptr: *const u8, len: u32) {
    let bytes = std::slice::from_raw_parts(ptr, len as usize).to_vec();
    machine(m).repl_inputs.push(super::ScheduledRepl { at_cycle, bytes });
}

/// Bytes the device has sent over CDC so far.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_serial_len(m: *mut Rp2040) -> u32 {
    machine(m).periph.cdc.serial_out.len() as u32
}

/// Copies up to `cap` of the CDC output bytes to `out`; returns the count.
///
/// # Safety
/// `m` from `rp_create`; `out` valid for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_serial_read(m: *mut Rp2040, out: *mut u8, cap: u32) -> u32 {
    let s = &machine(m).periph.cdc.serial_out;
    let n = s.len().min(cap as usize);
    std::ptr::copy_nonoverlapping(s.as_ptr(), out, n);
    n as u32
}

/// Copies bytes into SRAM at `offset` (synthetic images).
///
/// # Safety
/// `m` from `rp_create`; `ptr` valid for `len` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_sram_write(m: *mut Rp2040, offset: u32, ptr: *const u8, len: u32) -> i32 {
    let bytes = std::slice::from_raw_parts(ptr, len as usize);
    let mach = machine(m);
    let off = offset as usize;
    if off + bytes.len() > mach.sram.len() {
        return -1;
    }
    mach.sram[off..off + bytes.len()].copy_from_slice(bytes);
    0
}

/// Sets a core register (0..15) directly, as the harness does on rp2040js
/// (`core.registers[n] = v`, or `core.PC = v` for 15).
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_set_reg(m: *mut Rp2040, n: u32, value: u32) {
    machine(m).cpu.r[(n & 15) as usize] = value;
}

/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_schedule_input(m: *mut Rp2040, at_cycle: f64, pin: u32, level: u32) {
    machine(m).inputs.push(ScheduledInput { at_cycle, pin, level: level != 0 });
}

/// Runs up to `max_steps` steps; returns how many completed.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_run(m: *mut Rp2040, max_steps: f64) -> f64 {
    machine(m).run(max_steps as u64) as f64
}

/// 0 running, 1 peripheral, 2 USB DPRAM, 3 unaligned read, 4 out of range,
/// 5 break, 6 idle.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_stop_kind(m: *mut Rp2040) -> u32 {
    match machine(m).stop {
        None => 0,
        Some(Stop::Peripheral { .. }) => 1,
        Some(Stop::UsbDpram { .. }) => 2,
        Some(Stop::UnalignedRead { .. }) => 3,
        Some(Stop::OutOfRange { .. }) => 4,
        Some(Stop::Break { .. }) => 5,
        Some(Stop::Idle) => 6,
    }
}

/// The address (or break code) of the stop, and for a peripheral stop the
/// write flag in `rp_stop_write`.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_stop_address(m: *mut Rp2040) -> u32 {
    match machine(m).stop {
        Some(Stop::Peripheral { address, .. })
        | Some(Stop::UsbDpram { address })
        | Some(Stop::UnalignedRead { address })
        | Some(Stop::OutOfRange { address }) => address,
        Some(Stop::Break { code }) => code,
        _ => 0,
    }
}

/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_stop_write(m: *mut Rp2040) -> u32 {
    match machine(m).stop {
        Some(Stop::Peripheral { write, .. }) => write as u32,
        Some(Stop::UsbDpram { .. }) => 1,
        _ => 0,
    }
}

/// The stop's peripheral name, written as bytes to `out` (up to `cap`);
/// returns its length.
///
/// # Safety
/// `m` from `rp_create`; `out` valid for `cap` bytes.
#[no_mangle]
pub unsafe extern "C" fn rp_stop_name(m: *mut Rp2040, out: *mut u8, cap: u32) -> u32 {
    let name = match machine(m).stop {
        Some(Stop::Peripheral { name, .. }) => name,
        _ => "",
    };
    let n = name.len().min(cap as usize);
    std::ptr::copy_nonoverlapping(name.as_ptr(), out, n);
    n as u32
}

/// The compared state record, `STATE_WORDS` f64s:
/// 0..16 r0..r15, 16 xPSR, 17 MSP, 18 PSP, 19 PRIMASK, 20 CONTROL,
/// 21 cycles, 22 clock nanos, 23 waiting, 24 eventRegistered,
/// 25 pendingInterrupts, 26 enabledInterrupts, 27..31 priorities,
/// 31 NMI|PendSV|SVCall|SysTick pending bits, 32 interruptsUpdated,
/// 33 VTOR, 34 SHPR2, 35 SHPR3, 36 handler mode, 37 SIO GPIO out,
/// 38 SIO GPIO OE, 39 QSPI out, 40 QSPI OE, 41 spinlocks, 42 divider CSR,
/// 43 quotient, 44 remainder, 45 SysTick CSR flags, 46 SysTick counter,
/// 47 interpolator digest.
///
/// # Safety
/// `m` from `rp_create`; `out` valid for `STATE_WORDS` f64s.
#[no_mangle]
pub unsafe extern "C" fn rp_state(m: *mut Rp2040, out: *mut f64) {
    let s = std::slice::from_raw_parts_mut(out, STATE_WORDS);
    let mach = machine(m);
    let c = &mach.cpu;
    for i in 0..16 {
        s[i] = c.r[i] as f64;
    }
    s[16] = c.xpsr() as f64;
    s[17] = c.sp_main() as f64;
    s[18] = c.sp_process() as f64;
    s[19] = c.pm as u32 as f64;
    s[20] = ((if c.spsel { 2 } else { 0 }) | c.npriv as u32) as f64;
    s[21] = c.cycles as f64;
    s[22] = mach.clock.nanos;
    s[23] = c.waiting as u32 as f64;
    s[24] = c.event_registered as u32 as f64;
    s[25] = c.pending_interrupts as f64;
    s[26] = c.enabled_interrupts as f64;
    for p in 0..4 {
        s[27 + p] = c.interrupt_priorities[p] as f64;
    }
    // Slot 31 is reused: the four core-exception pending flags.
    s[31] = ((c.pending_nmi as u32)
        | ((c.pending_pendsv as u32) << 1)
        | ((c.pending_svcall as u32) << 2)
        | ((c.pending_systick as u32) << 3)) as f64;
    s[32] = c.interrupts_updated as u32 as f64;
    s[33] = c.vtor as f64;
    s[34] = c.shpr2 as f64;
    s[35] = c.shpr3 as f64;
    s[36] = c.handler_mode as u32 as f64;
    let sio = &mach.sio;
    s[37] = sio.gpio_value as f64;
    s[38] = sio.gpio_output_enable as f64;
    s[39] = sio.qspi_gpio_value as f64;
    s[40] = sio.qspi_gpio_output_enable as f64;
    s[41] = sio.spin_lock as f64;
    s[42] = sio.div_csr as f64;
    s[43] = sio.div_quotient as f64;
    s[44] = sio.div_remainder as f64;
    let ppb = &mach.ppb;
    s[45] = ((ppb.systick_count_flag as u32)
        | ((ppb.systick_clk_source as u32) << 1)
        | ((ppb.systick_int_enable as u32) << 2)
        | ((ppb.systick_timer.enabled as u32) << 3)) as f64;
    s[46] = ppb.systick_timer.counter(mach.clock.nanos) as f64;
    s[47] = interp_digest(mach) as f64;
    r2_state(mach, s);
}

fn fnv(values: impl IntoIterator<Item = u32>) -> u32 {
    let mut h = 0x811c_9dc5u32;
    for v in values {
        h = (h ^ v).wrapping_mul(0x0100_0193);
    }
    h
}

fn r2_state(mach: &mut Rp2040, s: &mut [f64]) {
    let t = &mach.periph.timer;
    let mut armed = 0u32;
    for (i, a) in t.alarms.iter().enumerate() {
        armed |= (a.armed as u32) << i;
        s[49 + i] = a.target_micros as f64;
    }
    s[48] = armed as f64;
    s[53] = t.int_raw as f64;
    s[54] = t.int_enable as f64;
    s[55] = t.int_force as f64;
    s[56] = t.latched_time_high as f64;
    s[57] = t.paused as u32 as f64;
    s[58] = mach.clock.alarms.len() as f64;
    let mut sum = 0.0;
    for &(_, at) in &mach.clock.alarms {
        sum += at;
    }
    s[59] = sum;
    let p = &mach.periph;
    s[60] = p.resets.reset as f64;
    s[61] = p.resets.wdsel as f64;
    s[62] = p.clocks.ctrl[5] as f64;
    s[63] = p.clocks.ctrl[4] as f64;
    s[64] = fnv((0..10).flat_map(|i| [p.clocks.ctrl[i], p.clocks.div[i]])) as f64;
    s[65] = p.xosc.ctrl as f64;
    s[66] = p.xosc.status_reg() as f64;
    s[67] = p.xosc.dormant as f64;
    s[68] = p.xosc.startup as f64;
    s[69] = p.xosc.count as f64;
    s[70] = p.psm.frce_on as f64;
    s[71] = p.psm.frce_off as f64;
    s[72] = p.psm.wdsel as f64;
    s[74] = p.watchdog.reason as f64;
    s[75] = fnv(p.watchdog.scratch) as f64;
    s[77] = p.watchdog.triggers as f64;
    s[78] = p.rtc.ctrl as f64;
    s[79] = p.rtc.setup0 as f64;
    s[80] = p.rtc.setup1 as f64;
    s[83] = p.nmi_mask as f64;
    s[84] = fnv(p.busctrl.perf_ctr.into_iter().chain(p.busctrl.perf_sel)) as f64;
    s[88] = (p.watchdog.alarm.enabled as u32 | ((p.watchdog.timer.enabled as u32) << 1)) as f64;
    s[90] = p.ssi.digest() as f64;
    s[91] = p.usb.digest() as f64;
    s[92] = fnv(mach.dpram.chunks_exact(4).map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))) as f64;
    s[93] = p.usb.alarm_digest() as f64;
    s[94] = p.cdc.digest() as f64;
    s[95] = p.cdc.wrapper_digest() as f64;
    s[96] = p.cdc.serial_out.len() as f64;
    s[97] = p.cdc.serial_hash() as f64;
    s[89] = (p.io.voltage_select[0] | (p.io.voltage_select[1] << 1)) as f64;
    s[85] = fnv(mach.sio.gpio.iter().map(|p| p.pad_value)) as f64;
    s[86] = fnv(mach.sio.qspi.iter().flat_map(|p| [p.pad_value, p.ctrl, p.irq_status])) as f64;
    s[87] = fnv(mach.sio.gpio.iter().flat_map(|p| [p.ctrl, p.irq_status, p.irq_enable_mask, p.irq_force_mask])) as f64;
    // Reads with no side effect on these blocks.
    s[73] = mach.watchdog_read(0x00) as f64;
    s[76] = mach.watchdog_read(0x2c) as f64;
    s[81] = mach.rtc_read(0x18) as f64;
    s[82] = mach.rtc_read(0x1c) as f64;
    r5_state(mach, s);
}

/// R5: the DMA channels and request lines, both PIO blocks and the pending
/// host-loop timers (slots 98..108).
fn r5_state(mach: &Rp2040, s: &mut [f64]) {
    let d = &mach.periph.dma;
    s[98] = d.channel_digest() as f64;
    s[99] = d.irq_digest() as f64;
    s[100] = d.dreq_digest() as f64;
    for p in 0..2 {
        let pio = &mach.periph.pio[p];
        s[101 + 3 * p] = pio.machine_digest() as f64;
        s[102 + 3 * p] = pio.fifo_digest() as f64;
        s[103 + 3 * p] = pio.reg_digest() as f64;
    }
    // The reference's queue holds opaque callbacks: only the count compares.
    s[107] = fnv([mach.periph.pio_timers.len() as u32]) as f64;
}

/// FNV-1a over the interpolators' registers, in a fixed order the harness
/// mirrors.
fn interp_digest(m: &Rp2040) -> u32 {
    let mut h = 0x811c_9dc5u32;
    for it in [&m.sio.interp0, &m.sio.interp1] {
        for v in [
            it.accum0, it.accum1, it.base0, it.base1, it.base2, it.ctrl0, it.ctrl1, it.result0, it.result1,
            it.result2, it.smresult0, it.smresult1,
        ] {
            h = (h ^ v).wrapping_mul(0x0100_0193);
        }
    }
    h
}

/// FNV-1a over SRAM as little-endian words.
pub fn ram_hash(m: &Rp2040) -> u32 {
    let mut h = 0x811c_9dc5u32;
    for w in m.sram.chunks_exact(4) {
        h = (h ^ u32::from_le_bytes([w[0], w[1], w[2], w[3]])).wrapping_mul(0x0100_0193);
    }
    h
}

/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_ram_hash(m: *mut Rp2040) -> u32 {
    ram_hash(machine(m))
}

/// A side-effect-free read for the harness's decodes.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_peek32(m: *mut Rp2040, address: u32) -> u32 {
    machine(m).peek_u32(address)
}

/// Completed steps since creation.
///
/// # Safety
/// `m` from `rp_create`.
#[no_mangle]
pub unsafe extern "C" fn rp_steps(m: *mut Rp2040) -> f64 {
    machine(m).steps as f64
}

/// Pin events from index `from` on, three f64s each (pin, state, nanos),
/// written to `out` up to `cap` events; returns the total count so far. The
/// harness reads only the events since its last checkpoint.
///
/// # Safety
/// `m` from `rp_create`; `out` valid for `3 * cap` f64s.
#[no_mangle]
pub unsafe extern "C" fn rp_pin_events_from(m: *mut Rp2040, from: u32, out: *mut f64, cap: u32) -> u32 {
    let mach = machine(m);
    let from = (from as usize).min(mach.pin_events.len());
    let n = (mach.pin_events.len() - from).min(cap as usize);
    let s = std::slice::from_raw_parts_mut(out, 3 * n);
    for (i, e) in mach.pin_events[from..].iter().take(n).enumerate() {
        s[3 * i] = e.pin as f64;
        s[3 * i + 1] = e.state as f64;
        s[3 * i + 2] = e.nanos;
    }
    mach.pin_events.len() as u32
}

/// Pin events so far, three f64s each (pin, state, nanos), written to `out`
/// up to `cap` events; returns the total count.
///
/// # Safety
/// `m` from `rp_create`; `out` valid for `3 * cap` f64s.
#[no_mangle]
pub unsafe extern "C" fn rp_pin_events(m: *mut Rp2040, out: *mut f64, cap: u32) -> u32 {
    let mach = machine(m);
    let n = mach.pin_events.len().min(cap as usize);
    let s = std::slice::from_raw_parts_mut(out, 3 * n);
    for (i, e) in mach.pin_events.iter().take(n).enumerate() {
        s[3 * i] = e.pin as f64;
        s[3 * i + 1] = e.state as f64;
        s[3 * i + 2] = e.nanos;
    }
    mach.pin_events.len() as u32
}
