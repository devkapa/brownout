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
