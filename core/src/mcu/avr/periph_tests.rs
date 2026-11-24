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
fn board(hex: &str) -> Cpu {
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

const SERIAL_COUNTER: &str = include_str!("../../../../test/sim/engine/__fixtures__/serial_counter.hex");
const SERIAL_ECHO: &str = include_str!("../../../../test/sim/engine/__fixtures__/serial_echo.hex");
const PERIPHERALS_OK: &str = include_str!("../../../../test/sim/engine/__fixtures__/peripherals_ok.hex");

#[test]
fn arduino_peripherals_include_the_usart_spi_twi_and_eeprom_at_their_reset_values() {
    // Datasheet reset values: UCSR0A has UDRE0 set, UCSR0C selects 8-bit
    // frames (UCSZ01:0 = 11), TWSR reads "no relevant state" (0xF8). avr8js
    // also raises TWINT at construction (its updateStatus call).
    let cpu = cpu_with(|c| c.attach_arduino_peripherals());
    assert!(cpu.p.usart.is_some() && cpu.p.spi.is_some() && cpu.p.twi.is_some() && cpu.p.eeprom.is_some());
    assert_eq!(cpu.data[0xc0], 0x20);
    assert_eq!(cpu.data[0xc2], 0x06);
    assert_eq!(cpu.data[0xb9], 0xf8);
    assert_eq!(cpu.data[0xbc], 0x80);
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
