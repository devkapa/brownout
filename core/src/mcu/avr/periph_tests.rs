//! AVR2 unit tests. Expected values come from the ATmega328P datasheet's
//! register descriptions and tables (port pin configuration, timer prescaler
//! and waveform modes, ADC conversion), derived in each test; the places where
//! avr8js departs from the datasheet are the `reference_quirk_*` tests.

use super::adc::MuxInput;
use super::cpu::Cpu;
use super::gpio::{PinOverride, PinState, PORT_B, PORT_D};

fn cpu_with(f: impl FnOnce(&mut Cpu)) -> Cpu {
    let mut cpu = Cpu::new(vec![0; 0x8000], 8192);
    f(&mut cpu);
    cpu
}

fn w(cpu: &mut Cpu, addr: i32, v: u8) {
    cpu.write_data(addr, v);
}

fn r(cpu: &mut Cpu, addr: i32) -> u8 {
    cpu.read_data(addr)
}

/// Runs `cycles` CPU cycles one at a time, ticking after each, as the timer
/// tests of the reference do.
fn advance(cpu: &mut Cpu, cycles: u64) {
    for _ in 0..cycles {
        cpu.cycles += 1;
        cpu.tick();
    }
}

#[test]
fn port_pin_configuration_matches_datasheet_table_14_1() {
    // DDxn, PORTxn -> I/O, pull-up: 00 input Hi-Z, 01 input pulled up,
    // 10 output low, 11 output high.
    let mut cpu = cpu_with(|c| c.attach_port(PORT_B));
    for (ddr, port, expect) in [
        (0, 0, PinState::Input),
        (0, 1, PinState::InputPullUp),
        (1, 0, PinState::Low),
        (1, 1, PinState::High),
    ] {
        w(&mut cpu, 0x24, ddr);
        w(&mut cpu, 0x25, port);
        assert_eq!(cpu.pin_state(PORT_B, 0), expect, "DDR={ddr} PORT={port}");
    }
}

#[test]
fn pin_register_reads_the_pin_for_inputs_and_the_output_for_outputs() {
    // PINxn is the pin's logic level: the external level for inputs, the
    // driven level for outputs.
    let mut cpu = cpu_with(|c| c.attach_port(PORT_B));
    w(&mut cpu, 0x24, 0x0f); // PB0-3 outputs
    w(&mut cpu, 0x25, 0x05);
    cpu.set_pin(PORT_B, 4, true);
    cpu.set_pin(PORT_B, 0, false); // an output ignores the outside level
    assert_eq!(cpu.data[0x23], 0x05 | 0x10);
}

#[test]
fn writing_a_one_to_pin_toggles_port_bit() {
    // Datasheet 14.2.2: writing a logic one to PINxn toggles PORTxn.
    let mut cpu = cpu_with(|c| c.attach_port(PORT_B));
    w(&mut cpu, 0x24, 0xff);
    w(&mut cpu, 0x25, 0b1010_1010);
    w(&mut cpu, 0x23, 0b0000_1111);
    assert_eq!(cpu.data[0x25], 0b1010_0101);
}

#[test]
fn int0_edge_modes_follow_eicra_table_12_2() {
    // ISC01:ISC00 = 00 low level, 01 any change, 10 falling, 11 rising.
    for (isc, rising_fires, falling_fires) in [(1u8, true, true), (2, false, true), (3, true, false)] {
        let mut cpu = cpu_with(|c| c.attach_port(PORT_D));
        w(&mut cpu, 0x3d, 1);
        w(&mut cpu, 0x69, isc);
        cpu.set_pin(PORT_D, 2, true);
        assert_eq!(cpu.data[0x3c] & 1 != 0, rising_fires, "rising, ISC0={isc}");
        w(&mut cpu, 0x3c, 1); // writing one clears the flag
        assert_eq!(cpu.data[0x3c] & 1, 0);
        cpu.set_pin(PORT_D, 2, false);
        assert_eq!(cpu.data[0x3c] & 1 != 0, falling_fires, "falling, ISC0={isc}");
    }
}

#[test]
fn timer0_prescalers_follow_table_14_9() {
    // CS02:0 = 1..5: clk/1, /8, /64, /256, /1024. TCNT0 advances by one every
    // `div` cycles once the timer is started.
    for (cs, div) in [(1u8, 1u64), (2, 8), (3, 64), (4, 256), (5, 1024)] {
        let mut cpu = cpu_with(|c| c.attach_timer(0));
        w(&mut cpu, 0x45, cs);
        cpu.cycles = 1;
        cpu.tick(); // the divider takes effect
        advance(&mut cpu, 5 * div);
        assert_eq!(r(&mut cpu, 0x46), 5, "CS0={cs}");
    }
}

#[test]
fn timer2_prescalers_follow_table_17_9() {
    // CS22:0 = 1..7: clk/1, /8, /32, /64, /128, /256, /1024 (timer 2 has its own set).
    for (cs, div) in [(1u8, 1u64), (2, 8), (3, 32), (4, 64), (5, 128), (6, 256), (7, 1024)] {
        let mut cpu = cpu_with(|c| c.attach_timer(2));
        w(&mut cpu, 0xb1, cs);
        cpu.cycles = 1;
        cpu.tick();
        advance(&mut cpu, 3 * div);
        assert_eq!(r(&mut cpu, 0xb2), 3, "CS2={cs}");
    }
}

#[test]
fn normal_mode_overflows_after_256_counts_and_sets_tov0() {
    let mut cpu = cpu_with(|c| c.attach_timer(0));
    w(&mut cpu, 0x45, 1);
    cpu.cycles = 1;
    cpu.tick();
    advance(&mut cpu, 255);
    assert_eq!(cpu.data[0x35] & 1, 0, "TOV0 clear at 255");
    advance(&mut cpu, 1);
    assert_eq!(cpu.data[0x35] & 1, 1, "TOV0 set when TCNT0 wraps");
    assert_eq!(r(&mut cpu, 0x46), 0);
}

#[test]
fn ctc_mode_counts_to_ocr0a_and_sets_ocf0a() {
    // WGM01 = 1: CTC, TOP = OCR0A; OCF0A is set when TCNT0 reaches OCR0A.
    let mut cpu = cpu_with(|c| c.attach_timer(0));
    w(&mut cpu, 0x44, 0x02);
    w(&mut cpu, 0x47, 9);
    w(&mut cpu, 0x45, 1);
    cpu.cycles = 1;
    cpu.tick();
    advance(&mut cpu, 8);
    assert_eq!(cpu.data[0x35] & 2, 0);
    advance(&mut cpu, 1);
    assert_eq!(cpu.data[0x35] & 2, 2, "OCF0A at TCNT0 = OCR0A");
    advance(&mut cpu, 1);
    assert_eq!(r(&mut cpu, 0x46), 0, "TCNT0 cleared after the match");
}

#[test]
fn fast_pwm_noninverting_duty_cycle_is_ocr_plus_one_over_256() {
    // Datasheet 14.7.3: non-inverting Fast PWM (COM0A = 2) clears OC0A on
    // compare match and sets it at BOTTOM, so OC0A is high for OCR0A + 1
    // counts of 256... avr8js's mapping is checked against the output pin
    // sampled every cycle at prescaler 1.
    let mut cpu = cpu_with(|c| {
        c.attach_port(PORT_D);
        c.attach_timer(0);
    });
    w(&mut cpu, 0x2a, 1 << 6); // PD6 = OC0A output
    w(&mut cpu, 0x47, 63);
    w(&mut cpu, 0x44, 0x80 | 0x03); // COM0A1, WGM01:0 = 3 (fast PWM)
    w(&mut cpu, 0x45, 1);
    cpu.cycles = 1;
    cpu.tick();
    let mut high = 0;
    let n = 256 * 4;
    advance(&mut cpu, 64); // settle into the first period
    for _ in 0..n {
        if cpu.data[0x29] & (1 << 6) != 0 {
            high += 1;
        }
        advance(&mut cpu, 1);
    }
    let duty = high as f64 / n as f64;
    assert!((duty - 64.0 / 256.0).abs() < 2.0 / 256.0, "duty {duty}");
}

#[test]
fn adc_conversion_matches_the_datasheet_formula_and_timing() {
    // ADC = floor(Vin * 1024 / Vref); a first conversion takes 25 ADC clocks,
    // later ones 13; the ADC clock is the CPU clock / prescaler (ADPS2:0 = 7: 128).
    let mut cpu = cpu_with(|c| c.attach_adc());
    cpu.set_adc_channel(2, 1.25);
    w(&mut cpu, 0x7c, 0x40 | 2); // AVCC reference, channel 2
    cpu.cycles = 1000;
    w(&mut cpu, 0x7a, 0x80 | 0x40 | 0x07); // ADEN | ADSC | /128
    assert_eq!(cpu.data[0x7a] & 0x40, 0x40, "ADSC set while converting");
    cpu.cycles = 1000 + 25 * 128 - 1;
    cpu.tick();
    assert_eq!(cpu.data[0x7a] & 0x40, 0x40, "still converting one cycle early");
    cpu.cycles += 1;
    cpu.tick();
    assert_eq!(cpu.data[0x7a] & 0x40, 0, "ADSC clears when done");
    assert_eq!(cpu.data[0x7a] & 0x10, 0x10, "ADIF set");
    let v = ((cpu.data[0x79] as u16) << 8) | cpu.data[0x78] as u16;
    assert_eq!(v, (1.25f64 * 1024.0 / 5.0).floor() as u16);
    // the next conversion takes 13 ADC clocks
    let t = cpu.cycles;
    w(&mut cpu, 0x7a, 0x80 | 0x40 | 0x07);
    cpu.cycles = t + 13 * 128;
    cpu.tick();
    assert_eq!(cpu.data[0x7a] & 0x40, 0);
}

#[test]
fn adc_left_adjust_and_reference_selection() {
    // ADLAR = 1 puts the 10-bit result in ADCH:ADCL[7:6]; REFS = 3 selects
    // the internal 1.1 V reference.
    let mut cpu = cpu_with(|c| c.attach_adc());
    cpu.set_adc_channel(0, 0.55);
    w(&mut cpu, 0x7c, 0xc0 | 0x20); // REFS1:0 = 11, ADLAR
    w(&mut cpu, 0x7a, 0x80 | 0x40 | 0x01);
    cpu.cycles = 25 * 2;
    cpu.tick();
    let raw = (0.55f64 / 1.1 * 1024.0).floor() as u16; // 512
    assert_eq!(cpu.data[0x79], (raw >> 2) as u8);
    assert_eq!(cpu.data[0x78], ((raw << 6) & 0xff) as u8);
}

// ---- named departures from the datasheet -------------------------------------------

#[test]
fn reference_quirk_pcmsk_write_enables_pin_change_interrupts_from_its_own_bits() {
    // The datasheet: PCICR bit PCIEn enables a pin-change interrupt. avr8js
    // has no PCICR hook; a PCMSK write re-evaluates every port's PCINT
    // interrupt against the written PCMSK value's bits 0-2.
    let mut cpu = cpu_with(|c| {
        c.attach_port(PORT_B);
    });
    cpu.data[0x68] = 0; // PCICR: PCINT0 disabled
    w(&mut cpu, 0x6b, 0x00);
    cpu.data[0x3b] = 1; // PCIF0 flagged
    w(&mut cpu, 0x6b, 0x01); // PCMSK0 = PCINT0 only
    assert_eq!(cpu.next_interrupt, 6, "queued although PCICR is 0");
}

#[test]
fn reference_quirk_tifr_write_stores_the_value_instead_of_clearing_on_one() {
    // The datasheet: a flag is cleared by writing a one and unaffected by a
    // zero. avr8js stores the written byte (a zero clears the flag register
    // bit) and clears the queued interrupt only for ones.
    let mut cpu = cpu_with(|c| c.attach_timer(0));
    w(&mut cpu, 0x6e, 1); // TOIE0
    cpu.set_interrupt_flag(cpu.p.timers[0].ovf);
    assert_eq!(cpu.next_interrupt, 0x20);
    w(&mut cpu, 0x35, 0); // writing zero
    assert_eq!(cpu.data[0x35] & 1, 0, "the flag bit is gone");
    assert_eq!(cpu.next_interrupt, 0x20, "but the interrupt is still pending");
}

#[test]
fn reference_quirk_adcsra_write_stores_adif_and_resets_conversion_time_only_from_zero() {
    let mut cpu = cpu_with(|c| c.attach_adc());
    // ADIF is cleared by writing a one on real hardware; here a zero clears it
    // and a one sets it.
    cpu.adc_complete(0);
    assert_eq!(cpu.data[0x7a] & 0x10, 0x10);
    w(&mut cpu, 0x7a, 0x80);
    assert_eq!(cpu.data[0x7a] & 0x10, 0, "writing ADIF = 0 cleared it");
    w(&mut cpu, 0x7a, 0x80 | 0x10);
    assert_eq!(cpu.data[0x7a] & 0x10, 0x10, "writing ADIF = 1 set it");
    // The first conversion after enabling takes 25 ADC clocks, but the
    // reference only restores 25 when ADCSRA was entirely zero.
    let mut cpu = cpu_with(|c| c.attach_adc());
    cpu.adc_complete(0); // conversion time is now 13
    cpu.data[0x7a] = 0x07; // prescaler set, ADC disabled
    w(&mut cpu, 0x7a, 0x87);
    assert_eq!(cpu.p.adc.as_ref().unwrap().conversion_cycles, 13);
    cpu.data[0x7a] = 0;
    w(&mut cpu, 0x7a, 0x87);
    assert_eq!(cpu.p.adc.as_ref().unwrap().conversion_cycles, 25);
}

#[test]
fn reference_quirk_tccr1c_keeps_the_force_compare_bits() {
    // FOC1A/FOC1B are strobes and read as zero; the reference stores them.
    let mut cpu = cpu_with(|c| c.attach_timer(1));
    w(&mut cpu, 0x82, 0xc0);
    assert_eq!(cpu.data[0x82], 0xc0);
}

#[test]
fn reference_quirk_adc_reserved_reference_reads_as_avcc_and_input_clamps() {
    // REFS = 2 is reserved; the reference uses AVCC. A voltage above the
    // reference saturates at 1023.
    let mut cpu = cpu_with(|c| c.attach_adc());
    cpu.set_adc_channel(0, 5.0);
    w(&mut cpu, 0x7c, 0x80);
    w(&mut cpu, 0x7a, 0xc1);
    cpu.cycles = 50;
    cpu.tick();
    assert_eq!(((cpu.data[0x79] as u16) << 8) | cpu.data[0x78] as u16, 1023);
    cpu.p.log_overrides = true;
    cpu.set_adc_channel(0, 2.5);
    w(&mut cpu, 0x7a, 0xc1);
    assert_eq!(cpu.p.adc_reads, vec![MuxInput::SingleEnded(0)]);
}

#[test]
fn reference_quirk_timer_override_is_recorded_only_for_attached_ports() {
    // gpioByPort[...]?.timerOverridePin: a compare output whose port was never
    // constructed is silently ignored.
    let mut cpu = cpu_with(|c| c.attach_timer(0));
    cpu.p.log_overrides = true;
    w(&mut cpu, 0x44, 0x80); // COM0A1 with no port D attached
    assert!(cpu.p.overrides.is_empty());
    cpu.attach_port(PORT_D);
    w(&mut cpu, 0x44, 0x00);
    w(&mut cpu, 0x44, 0x80);
    assert!(cpu.p.overrides.contains(&(PORT_D, 6, PinOverride::Enable)));
}

// ---- USART, SPI, TWI and EEPROM as `ArduinoMcu` builds them ----------------

/// Program words from an Intel HEX image (data records only), as brownout's
/// `parseIntelHex` lays them out in a 32 KiB flash.
fn hex_words(hex: &str) -> Vec<u16> {
    let mut bytes = vec![0u8; 0x8000 * 2];
    for line in hex.lines().map(str::trim).filter(|l| l.starts_with(':')) {
        let b: Vec<u8> = (1..line.len()).step_by(2).map(|i| u8::from_str_radix(&line[i..i + 2], 16).unwrap()).collect();
        if b[3] != 0 {
            continue;
        }
        let addr = ((b[1] as usize) << 8) | b[2] as usize;
        bytes[addr..addr + b[0] as usize].copy_from_slice(&b[4..4 + b[0] as usize]);
    }
    bytes.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect()
}

/// A board running `hex`, built as `new ArduinoMcu(hex)` builds it.
pub(super) fn board(hex: &str) -> Cpu {
    let mut cpu = Cpu::new(hex_words(hex), 8192);
    cpu.attach_arduino_peripherals();
    cpu
}

fn run(cpu: &mut Cpu, cycles: u64) {
    let end = cpu.cycles + cycles;
    while cpu.cycles < end {
        super::instr::avr_instruction(cpu);
        cpu.tick();
    }
}

pub(super) const SERIAL_COUNTER: &str = include_str!("../../../../test/sim/engine/__fixtures__/serial_counter.hex");
const SERIAL_ECHO: &str = include_str!("../../../../test/sim/engine/__fixtures__/serial_echo.hex");
const PERIPHERALS_OK: &str = include_str!("../../../../test/sim/engine/__fixtures__/peripherals_ok.hex");

#[test]
fn arduino_peripherals_include_the_usart_spi_twi_and_eeprom_at_their_reset_values() {
    // Datasheet reset values: UCSR0A has UDRE0 set, UCSR0C selects 8-bit
    // frames (UCSZ01:0 = 11), TWSR reads "no relevant state" (0xF8) and
    // TWCR is 0 (avr8js raises TWINT at construction; ArduinoMcu's timed TWI
    // clears it).
    let cpu = cpu_with(|c| c.attach_arduino_peripherals());
    assert!(cpu.p.usart.is_some() && cpu.p.spi.is_some() && cpu.p.twi.is_some() && cpu.p.eeprom.is_some());
    assert_eq!(cpu.data[0xc0], 0x20);
    assert_eq!(cpu.data[0xc2], 0x06);
    assert_eq!(cpu.data[0xb9], 0xf8);
    assert_eq!(cpu.data[0xbc], 0x00);
    assert_eq!(cpu.eeprom_memory(), &[0xff; 1024][..]);
}

#[test]
fn the_comms_registers_count_as_ported() {
    for a in [0x3f, 0x40, 0x41, 0x42, 0x4c, 0x4d, 0x4e, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xc0, 0xc1, 0xc2, 0xc4, 0xc5, 0xc6] {
        assert!(super::gpio::is_ported_register(a), "{a:#x}");
    }
    assert!(!super::gpio::is_ported_register(0xc3));
}

#[test]
fn a_sketch_printing_past_the_transmit_buffer_keeps_running() {
    // 9600 baud, 8N1: a frame is 10 bits of 16 * (103 + 1) cycles = 16640
    // cycles. 0.1 s is 1.6M cycles, about 96 frames: past Serial's 64-byte
    // buffer, which only drains if the UART reports each byte sent.
    let mut cpu = board(SERIAL_COUNTER);
    run(&mut cpu, 1_600_000);
    let out = cpu.serial_drain();
    let text = String::from_utf8_lossy(&out);
    assert!(text.starts_with("count 0\r\ncount 1\r\ncount 2\r\n"), "{text:?}");
    assert!((90..=97).contains(&out.len()), "{} bytes", out.len());
    assert!(cpu.serial_drain().is_empty(), "a drain empties the log");
}

#[test]
fn undrained_serial_output_keeps_only_the_newest_bytes_and_counts_the_rest() {
    // A board that prints forever with nothing draining it must not grow
    // without bound; `ArduinoMcu` keeps the same ring.
    let mut all = board(SERIAL_COUNTER);
    let mut capped = board(SERIAL_COUNTER);
    capped.p.usart.as_mut().unwrap().tx_capacity = 64;
    run(&mut all, 1_600_000);
    run(&mut capped, 1_600_000);
    let sent = all.serial_drain();
    assert!(sent.len() > 64);
    assert_eq!(capped.serial_drain(), sent[sent.len() - 64..].to_vec());
    assert_eq!(capped.serial_dropped(), (sent.len() - 64) as u64);
    assert_eq!(all.serial_dropped(), 0);
    // The lock-step record still sees every byte.
    assert_eq!(capped.p.usart.as_ref().unwrap().tx_count, sent.len() as u64);
    assert_eq!(capped.p.usart.as_ref().unwrap().tx_hash, all.p.usart.as_ref().unwrap().tx_hash);
}

#[test]
fn the_serial_output_ring_holds_16_kib_by_default() {
    assert_eq!(board(SERIAL_COUNTER).p.usart.as_ref().unwrap().tx_capacity, 16384);
}

#[test]
fn recorded_serial_lines_keep_only_the_newest() {
    let mut all = board(SERIAL_COUNTER);
    let mut capped = board(SERIAL_COUNTER);
    all.p.usart.as_mut().unwrap().line_buffer = Some(String::new());
    let u = capped.p.usart.as_mut().unwrap();
    u.line_buffer = Some(String::new());
    u.lines_capacity = 3;
    run(&mut all, 1_600_000);
    run(&mut capped, 1_600_000);
    let every: Vec<String> = all.p.usart.as_ref().unwrap().lines.iter().cloned().collect();
    assert!(every.len() > 3 && every[0] == "count 0\r", "{every:?}");
    let u = capped.p.usart.as_ref().unwrap();
    assert_eq!(u.lines, every[every.len() - 3..].to_vec());
    assert_eq!(u.lines_dropped, every.len() as u64 - 3);
}

#[test]
fn typed_bytes_reach_serial_read_one_frame_apart() {
    let mut cpu = board(SERIAL_ECHO);
    run(&mut cpu, 200_000); // through setup()
    cpu.serial_drain();
    cpu.serial_write(b"abc\n");
    // 115200 baud: UBRR 16 with U2X (Arduino's choice), 10 * 8 * 17 = 1360
    // cycles a frame; four frames in, four out.
    run(&mut cpu, 20_000);
    assert_eq!(cpu.serial_drain(), b"ABC\n".to_vec());
}

#[test]
fn eeprom_wire_and_spi_calls_complete() {
    // Wire reports 2 (address NACK: no part on the bus), SPI reads back 0
    // (nothing drives MISO), EEPROM keeps both writes.
    let mut cpu = board(PERIPHERALS_OK);
    run(&mut cpu, 800_000);
    let text = String::from_utf8(cpu.serial_drain()).unwrap();
    assert_eq!(text, "eeprom 42,43 i2c 2 spi 0\r\n");
    assert_eq!(&cpu.eeprom_memory()[..2], &[42, 43]);
}

#[test]
fn eeprom_contents_carry_into_a_new_board() {
    // EEPROM is non-volatile: the host builds the next board's EEPROM from the
    // last one's contents, as ArduinoMcu keeps one backend across reset().
    let mut cpu = board(PERIPHERALS_OK);
    run(&mut cpu, 800_000);
    let kept = cpu.eeprom_memory().to_vec();
    let mut next = Cpu::new(hex_words(PERIPHERALS_OK), 8192);
    next.attach_arduino_peripherals_with_eeprom(kept.clone());
    assert_eq!(next.eeprom_memory(), &kept[..]);
}

#[test]
fn serial_input_waits_while_the_receiver_is_off() {
    // ArduinoMcu's feed: writeByte refuses while RXEN is clear, so the bytes
    // stay queued until the next write after the sketch enables it.
    let mut cpu = cpu_with(|c| c.attach_arduino_peripherals());
    cpu.serial_write(b"x");
    assert_eq!(cpu.p.usart.as_ref().unwrap().rx_queue.len(), 1);
    w(&mut cpu, 0xc4, 16);
    w(&mut cpu, 0xc1, 0x10); // RXEN
    cpu.serial_write(b"y");
    assert_eq!(cpu.p.usart.as_ref().unwrap().rx_queue.len(), 1, "x went to the receiver; y waits behind it");
    advance(&mut cpu, 16 * 17 * 10);
    assert_eq!(r(&mut cpu, 0xc6), b'x');
}

#[test]
fn harness_inputs_due_at_one_boundary_are_judged_against_its_cycle_count() {
    // An EERE write halts the CPU four cycles inside the input itself. The JS
    // side decides which inputs are due once per instruction boundary, so an
    // input due inside those four cycles waits for the next boundary there.
    use super::harness::{Input, Machine, ScheduledInput};
    let mut m = Machine::new(vec![0; 64]);
    m.cpu.attach_arduino_peripherals();
    m.inputs.push(ScheduledInput { at_cycle: 0, input: Input::Write { addr: 0x3f, value: 0x01 } });
    m.inputs.push(ScheduledInput { at_cycle: 3, input: Input::Write { addr: 0x40, value: 0x5a } });
    m.step();
    assert_eq!(m.cpu.cycles, 5, "EERE's 4 cycles and one NOP");
    assert_ne!(m.cpu.data[0x40], 0x5a, "the second input waits for the next boundary");
    m.step();
    assert_eq!(m.cpu.data[0x40], 0x5a);
}

#[test]
#[should_panic(expected = "1024")]
fn a_board_eeprom_is_exactly_1024_bytes() {
    let mut cpu = Cpu::new(vec![0; 64], 8192);
    cpu.attach_arduino_peripherals_with_eeprom(vec![0xff; 512]);
}

// Bus events for protocol decoders (`ArduinoMcu.drainBusEvents`): the same
// traffic, in the same order, with the same cycles.

#[test]
fn bus_events_record_i2c_spi_and_uart_traffic_in_order() {
    use super::bus::BusEvent;
    let mut cpu = board(PERIPHERALS_OK);
    run(&mut cpu, 800_000);
    let events = cpu.bus_drain();
    assert_eq!(cpu.bus_dropped(), 0);
    assert!(events.windows(2).all(|w| w[0].cycle() <= w[1].cycle()));
    let i2c: Vec<&BusEvent> = events.iter().filter(|e| matches!(e, BusEvent::I2c { .. })).collect();
    assert_eq!(i2c.len(), 3, "{i2c:?}");
    assert!(matches!(i2c[0], BusEvent::I2c { kind: super::bus::I2cKind::Start, .. }));
    assert!(matches!(i2c[1], BusEvent::I2c { kind: super::bus::I2cKind::Address { address: 0x3c, read: false, ack: false }, .. }));
    assert!(matches!(i2c[2], BusEvent::I2c { kind: super::bus::I2cKind::Stop, .. }));
    let spi: Vec<&BusEvent> = events.iter().filter(|e| matches!(e, BusEvent::Spi { .. })).collect();
    assert_eq!(spi.len(), 1);
    assert!(matches!(spi[0], BusEvent::Spi { mosi: 0x5a, miso: 0, cycles, .. } if *cycles > 0));
    let text: Vec<u8> = events
        .iter()
        .filter_map(|e| match e {
            BusEvent::Uart { tx: true, value, cycles: 1360, .. } => Some(*value),
            BusEvent::Uart { .. } => panic!("unexpected frame {e:?}"),
            _ => None,
        })
        .collect();
    assert_eq!(String::from_utf8_lossy(&text), "eeprom 42,43 i2c 2 spi 0\r\n");
    assert!(cpu.bus_drain().is_empty());
}

#[test]
fn bus_events_record_received_bytes_as_they_start_to_arrive() {
    use super::bus::BusEvent;
    let mut cpu = board(SERIAL_ECHO);
    run(&mut cpu, 320_000);
    cpu.bus_drain();
    cpu.serial_write(b"hi");
    run(&mut cpu, 800_000);
    let rx: Vec<(u64, u8, i64)> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match e {
            BusEvent::Uart { tx: false, cycle, value, cycles } => Some((*cycle, *value, *cycles)),
            _ => None,
        })
        .collect();
    assert_eq!(rx.iter().map(|r| r.1).collect::<Vec<_>>(), b"hi".to_vec());
    assert!(rx[1].0 - rx[0].0 >= rx[0].2 as u64);
}

#[test]
fn undrained_bus_events_keep_only_the_newest() {
    let mut all = board(SERIAL_COUNTER);
    let mut capped = board(SERIAL_COUNTER);
    capped.p.bus.capacity = 64;
    run(&mut all, 8_000_000);
    run(&mut capped, 8_000_000);
    let every = all.bus_drain();
    assert!(every.len() > 64);
    assert_eq!(capped.bus_drain(), every[every.len() - 64..].to_vec());
    assert_eq!(capped.bus_dropped(), every.len() as u64 - 64);
    assert_eq!(board(SERIAL_COUNTER).p.bus.capacity, 4096);
}

// The 328P's bus timing where avr8js has none (ArduinoMcu's avr-bus-timing.ts,
// which the lock-step harness runs too): I2C steps at the SCL rate, and a
// double-buffered USART transmitter.

const TWI_TIMING: &str = include_str!("../../../../test/sim/engine/__fixtures__/twi_timing.hex");
const UART_BUFFER: &str = include_str!("../../../../test/sim/engine/__fixtures__/uart_buffer.hex");

#[test]
fn twi_steps_take_their_bus_time_at_the_scl_rate() {
    use super::bus::{BusEvent, I2cKind};
    let mut cpu = board(TWI_TIMING);
    run(&mut cpu, 32_000);
    let i2c: Vec<(u64, i64, I2cKind)> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match *e {
            BusEvent::I2c { cycle, cycles, kind } => Some((cycle, cycles, kind)),
            _ => None,
        })
        .collect();
    assert_eq!(i2c.len(), 6, "{i2c:?}");
    // TWBR 72 is 100 kHz, 160 cycles a period; TWBR 12 is 400 kHz, 40.
    for (n, scl) in [(0usize, 160i64), (3, 40)] {
        let (start, address, stop) = (i2c[n], i2c[n + 1], i2c[n + 2]);
        assert_eq!(start.2, I2cKind::Start, "a plain TWSR store must not wipe the idle status");
        assert_eq!((start.1, address.1, stop.1), (scl / 2, 9 * scl, scl));
        assert!(matches!(address.2, I2cKind::Address { address: 0x3c, read: false, ack: false }));
        assert_eq!(stop.2, I2cKind::Stop);
        let after_start = address.0 as i64 - start.0 as i64 - scl / 2;
        let after_address = stop.0 as i64 - address.0 as i64 - 9 * scl;
        assert!((0..40).contains(&after_start), "{after_start}");
        assert!((0..40).contains(&after_address), "{after_address}");
    }
    assert!(i2c[3].0 - i2c[2].0 >= 160, "the next START waits for the STOP to finish");
}

#[test]
fn usart_transmitter_buffers_one_byte_and_sends_frames_back_to_back() {
    use super::bus::BusEvent;
    const PORTB: usize = 0x25;
    let mut cpu = board(UART_BUFFER);
    // Run until A's frame has started, then a little: B must already be accepted into UDR.
    let mut first = None;
    while first.is_none() {
        run(&mut cpu, 1);
        first = cpu.p.bus.events.iter().find_map(|e| match *e {
            BusEvent::Uart { tx: true, cycle, .. } => Some(cycle),
            _ => None,
        });
    }
    let a = first.unwrap();
    run(&mut cpu, 100);
    assert_eq!(cpu.data[PORTB], 2, "B waits in UDR while A is on the wire");
    run(&mut cpu, 32_000);
    assert_eq!(cpu.data[PORTB], 4);
    let tx: Vec<(u64, u8)> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match *e {
            BusEvent::Uart { tx: true, cycle, value, .. } => Some((cycle, value)),
            _ => None,
        })
        .collect();
    assert_eq!(tx, vec![(a, b'A'), (a + 1360, b'B'), (a + 2720, b'C')]);
    assert_eq!(cpu.serial_drain(), b"ABC".to_vec());
}

const UART_UDRIE: &str = include_str!("../../../../test/sim/engine/__fixtures__/uart_udrie.hex");

#[test]
fn usart_data_register_empty_interrupt_keeps_firing_while_udr_has_room() {
    use super::bus::BusEvent;
    let mut cpu = board(UART_UDRIE);
    run(&mut cpu, 32_000);
    assert_eq!(cpu.serial_drain(), b"ABCDE".to_vec());
    let starts: Vec<u64> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match *e {
            BusEvent::Uart { tx: true, cycle, .. } => Some(cycle),
            _ => None,
        })
        .collect();
    assert!(starts.windows(2).all(|w| w[1] - w[0] == 1360), "{starts:?}");
}

const UART_1200: &str = include_str!("../../../../test/sim/engine/__fixtures__/uart_1200.hex");

#[test]
fn usart_frames_are_dated_from_ubrr_exactly_at_any_baud_rate() {
    use super::bus::BusEvent;
    let mut cpu = board(UART_1200);
    run(&mut cpu, 160_000);
    // 1200 baud with U2X: UBRR 1666, (1666 + 1) x 8 x 10 cycles a frame.
    let frames: Vec<(u8, i64)> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match *e {
            BusEvent::Uart { tx: true, value, cycles, .. } => Some((value, cycles)),
            _ => None,
        })
        .collect();
    assert_eq!(frames, vec![(b'x', 1667 * 8 * 10)]);
}

const UART_FLAGS: &str = include_str!("../../../../test/sim/engine/__fixtures__/uart_flags.hex");

#[test]
fn usart_ignores_a_write_while_udr_is_full_and_starting_a_frame_leaves_txc_set() {
    let mut cpu = board(UART_FLAGS);
    run(&mut cpu, 32_000);
    assert_eq!(cpu.serial_drain(), b"ABD".to_vec(), "C was written while UDR held B");
    assert_eq!(cpu.data[0x25] & 1, 1, "TXC still set after D's frame started");
}

const TWI_FLAGS: &str = include_str!("../../../../test/sim/engine/__fixtures__/twi_flags.hex");

#[test]
fn twint_rises_only_where_the_datasheet_has_it() {
    // PORTB bit 0: clear out of reset; bit 1: a write of zero leaves it set;
    // bit 2: no TWINT after a STOP, TWSR 0xF8; bit 3: the program finished.
    let mut cpu = board(TWI_FLAGS);
    run(&mut cpu, 32_000);
    assert_eq!(cpu.data[0x25], 0x0f);
}

#[test]
fn a_start_after_a_stop_waits_out_the_bus_free_time() {
    use super::bus::{BusEvent, I2cKind};
    let mut cpu = board(TWI_FLAGS);
    run(&mut cpu, 32_000);
    let i2c: Vec<(u64, i64, I2cKind)> = cpu
        .bus_drain()
        .iter()
        .filter_map(|e| match *e {
            BusEvent::I2c { cycle, cycles, kind } => Some((cycle, cycles, kind)),
            _ => None,
        })
        .collect();
    let kinds: Vec<I2cKind> = i2c.iter().map(|e| e.2).collect();
    assert_eq!(kinds, vec![I2cKind::Start, I2cKind::Stop, I2cKind::Start, I2cKind::Stop, I2cKind::Start]);
    // t_BUF is 4.7 us up to 100 kHz: 75.2 cycles at 16 MHz, rounded up.
    let free = i2c[4].0 as i64 - (i2c[3].0 as i64 + i2c[3].1);
    assert!((76..80).contains(&free), "{free}");
}

const UART_RX: &str = include_str!("../../../../test/sim/engine/__fixtures__/uart_rx.hex");

/// uart_rx with `text` typed once its receiver is on; returns the board and
/// the cycle the first frame started arriving on.
fn receive(text: &[u8]) -> (Cpu, u64) {
    use super::bus::BusEvent;
    let mut cpu = board(UART_RX);
    run(&mut cpu, 8_000);
    cpu.bus_drain();
    cpu.serial_write(text);
    let start = cpu
        .bus_drain()
        .iter()
        .find_map(|e| match *e {
            BusEvent::Uart { tx: false, cycle, .. } => Some(cycle),
            _ => None,
        })
        .unwrap();
    (cpu, start)
}

#[test]
fn rxc_rises_at_the_stop_bit_sample_before_the_frame_ends() {
    let (mut cpu, start) = receive(b"W");
    // 115200 with U2X: 8 samples a bit, 17 cycles each; nine bits in, then
    // samples 4, 5 and 6 of the stop bit.
    let rxc = (9 * 8 + 5) * 17;
    while cpu.data[0x25] != 1 {
        run(&mut cpu, 1);
    }
    let seen = cpu.cycles - start;
    assert!(seen >= rxc && seen < 1360, "{seen}");
}

#[test]
fn the_receiver_holds_two_frames_and_a_third_and_a_fourth_overruns_the_third() {
    let (mut cpu, _) = receive(b"WXYZ");
    run(&mut cpu, 160_000);
    assert_eq!(cpu.data[0x25], 2);
    let res = &cpu.data[0x300..0x307];
    assert_eq!(res[0], 3);
    let got: Vec<(u8, bool)> = (0..3).map(|i| (res[2 + 2 * i], res[1 + 2 * i] & 0x08 != 0)).collect();
    assert_eq!(got, vec![(b'W', false), (b'X', false), (b'Z', true)]);
}

#[test]
fn disabling_the_receiver_flushes_its_fifo() {
    let mut cpu = cpu_with(|c| c.attach_arduino_peripherals());
    w(&mut cpu, 0xc4, 16);
    w(&mut cpu, 0xc0, 0x02);
    w(&mut cpu, 0xc1, 0x10); // RXEN
    cpu.serial_write(b"ab");
    advance(&mut cpu, 2 * 1360 + 10);
    assert_eq!(r(&mut cpu, 0xc0) & 0x80, 0x80);
    w(&mut cpu, 0xc1, 0x00);
    assert_eq!(r(&mut cpu, 0xc0) & 0x80, 0, "RXC clears with the FIFO");
    w(&mut cpu, 0xc1, 0x10);
    assert_eq!(r(&mut cpu, 0xc0) & 0x80, 0);
}

const FLAGS_ISR: &str = include_str!("../../../../test/sim/engine/__fixtures__/flags_isr.hex");

#[test]
fn twint_and_udre_stay_set_when_their_interrupts_are_taken() {
    // PORTB bit 0: TWINT set inside its handler; bit 1: still set after it
    // wrote TWCR with TWINT 0; bit 2: UDRE set inside its handler; bit 3: done.
    let mut cpu = board(FLAGS_ISR);
    run(&mut cpu, 32_000);
    assert_eq!(cpu.data[0x25], 0x0f);
}

