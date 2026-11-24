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

/// Constructs ports B, C, D, timers 0-2, the ADC, USART0, TWI, SPI and the
/// EEPROM, as `new ArduinoMcu` does.
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

/// Moves up to `cap` bus events (`ArduinoMcu.drainBusEvents().events`) to
/// `out`, six f64s each: a code (UART sent 0, received 1; I2C start 2,
/// restart 3, stop 4, address 5, write 6, read 7; SPI 8), the start cycle,
/// the length in cycles, then address, read and ack, or value and ack, or
/// MOSI and MISO, or the UART value, with zeros after. Returns how many.
///
/// # Safety
/// `m` from `avr_create`; `out` valid for `cap * 6` f64s.
#[no_mangle]
pub unsafe extern "C" fn avr_drain_bus(m: *mut Machine, out: *mut f64, cap: u32) -> u32 {
    use super::bus::{BusEvent, I2cKind};
    let log = &mut machine(m).cpu.p.bus.events;
    let n = log.len().min(cap as usize);
    let buf = std::slice::from_raw_parts_mut(out, n * 6);
    for (i, e) in log.drain(..n).enumerate() {
        let row: [f64; 6] = match e {
            BusEvent::Uart { cycle, tx, value, cycles } => [if tx { 0.0 } else { 1.0 }, cycle as f64, cycles as f64, value as f64, 0.0, 0.0],
            BusEvent::Spi { cycle, mosi, miso, cycles } => [8.0, cycle as f64, cycles as f64, mosi as f64, miso as f64, 0.0],
            BusEvent::I2c { cycle, cycles, kind } => {
                let (code, x, y, z) = match kind {
                    I2cKind::Start => (2, 0, 0, 0),
                    I2cKind::Restart => (3, 0, 0, 0),
                    I2cKind::Stop => (4, 0, 0, 0),
                    I2cKind::Address { address, read, ack } => (5, address, read as u8, ack as u8),
                    I2cKind::Write { value, ack } => (6, value, ack as u8, 0),
                    I2cKind::Read { value, ack } => (7, value, ack as u8, 0),
                };
                [code as f64, cycle as f64, cycles as f64, x as f64, y as f64, z as f64]
            }
        };
        buf[i * 6..i * 6 + 6].copy_from_slice(&row);
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

/// Schedules `ArduinoMcu.writeSerialInput([byte])`.
///
/// # Safety
/// `m` from `avr_create`.
#[no_mangle]
pub unsafe extern "C" fn avr_schedule_serial(m: *mut Machine, at_cycle: f64, byte: u32) {
    machine(m).inputs.push(ScheduledInput { at_cycle: at_cycle as u64, input: Input::SerialRx { byte: byte as u8 } });
}

/// Puts a powered SSD1306 (`new Ssd1306({ address })`, 128x64) on the
/// board's TWI, after any parts already there.
///
/// # Safety
/// `m` from `avr_create`, after `avr_attach`.
#[no_mangle]
pub unsafe extern "C" fn avr_attach_ssd1306(m: *mut Machine, address: u32) {
    let mut d = super::ssd1306::Ssd1306::new(address as u8, 64);
    d.powered = true;
    machine(m).cpu.p.parts.i2c.push(super::parts::I2cPart::Ssd1306(d));
}

/// Puts a powered BMP280 at `address` on the board's TWI with its ambient
/// temperature (degC) and pressure (Pa), set as its host sets them.
///
/// # Safety
/// `m` from `avr_create`, after `avr_attach`.
#[no_mangle]
pub unsafe extern "C" fn avr_attach_bmp280(m: *mut Machine, address: u32, celsius: f64, pascals: f64) {
    let cpu = &mut machine(m).cpu;
    let mut b = super::bmp280::Bmp280::new(address as u8);
    b.powered = true;
    b.set_ambient(celsius, pascals, cpu.board_seconds());
    cpu.p.parts.i2c.push(super::parts::I2cPart::Bmp280(b));
}

/// Puts a powered DHT (`variant` 22 or 11), past its power-up second, on
/// board pin (`port` 0 B, 1 C, 2 D; `bit`) with its ambient humidity (%RH)
/// and temperature (degC), and the module's pull-up as the circuit's level
/// there.
///
/// # Safety
/// `m` from `avr_create`, after `avr_attach`.
#[no_mangle]
pub unsafe extern "C" fn avr_attach_dht(m: *mut Machine, port: u32, bit: u32, variant: u32, humidity: f64, celsius: f64) {
    let cpu = &mut machine(m).cpu;
    let mut dht = super::dht::DhtSensor::new(variant as u8);
    dht.powered = true;
    dht.powered_for = 2.0;
    dht.set_ambient(humidity, celsius);
    let mut slots = cpu.p.parts.pins.clone();
    slots.push(super::parts::PinSlot { port: port as u8, bit: bit as u8, dht });
    cpu.set_pin_targets(slots);
    cpu.set_input_bit(port as u8, bit as u8, true);
}
