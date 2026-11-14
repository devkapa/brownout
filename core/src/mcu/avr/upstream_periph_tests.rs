//! avr8js 0.21.0's gpio, adc and timer suites, ported to the Rust port (MIT,
//! Copyright (c) Uri Shaked and contributors; see NOTICE). Upstream files
//! (github.com/wokwi/avr8js, tag v0.21.0, 898f76c): `src/peripherals/gpio.spec.ts`,
//! `adc.spec.ts` and `timer.spec.ts`.
//!
//! Converted statement by statement from the TypeScript (a one-off script
//! that only read the spec text; nothing from the upstream checkout was run)
//! and reviewed. Spies become the port's recorded calls: `vi.fn()` listeners
//! the GPIO event list, `spyOn(port, 'timerOverridePin')` `cpu.p.overrides`,
//! `spyOn(adc, 'onADCRead')` `cpu.p.adc_reads`. Upstream's assembler is replaced by
//! `asm_program` below (straight-line code, labels, `_REPLACE`).
//!
//! Not ported: the five timer tests that build a timer from a custom config
//! (an OCRC compare unit or ATtiny-style bit positions); `ArduinoMcu` only
//! constructs the ATmega328P timers, which have neither. The two GPIO tests
//! that assert inside a listener are written out by hand (the port records
//! `pinState` at listener time in the event).

#![allow(unused_mut, unused_variables, dead_code)]

use super::adc::MuxInput;
use super::cpu::Cpu;
use super::gpio::{PinOverride, PinState, PORT_B, PORT_C, PORT_D};
use super::instr::avr_instruction;
use super::upstream_tests::asm_line;
use std::collections::HashMap;

struct Asm {
    program: Vec<u16>,
    instruction_count: usize,
    nop_count: usize,
    /// Byte addresses.
    labels: HashMap<String, usize>,
}

fn asm_program(src: &str) -> Asm {
    let mut subst: Vec<(String, String)> = Vec::new();
    let mut lines: Vec<(Option<String>, String)> = Vec::new(); // (label, instruction)
    let mut pending: Option<String> = None;
    for raw in src.lines() {
        let mut l = raw.split(';').next().unwrap().trim().to_string();
        if l.is_empty() {
            continue;
        }
        if let Some(rest) = l.strip_prefix("_REPLACE ") {
            let (a, b) = rest.split_once(',').unwrap();
            subst.push((a.trim().to_string(), b.trim().to_string()));
            continue;
        }
        let mut label = pending.take();
        if let Some((a, b)) = l.split_once(':') {
            label = Some(a.trim().to_string());
            l = b.trim().to_string();
            if l.is_empty() {
                pending = label;
                continue;
            }
        }
        for (a, b) in &subst {
            l = l
                .split(|c: char| c == ' ' || c == ',')
                .map(|w| if w == a { b.as_str() } else { w })
                .collect::<Vec<_>>()
                .join(" ")
                .replace(" ", " ");
        }
        // re-insert the comma after the first operand: "STS ADMUX r24" -> "STS 124, r24"
        lines.push((label, l));
    }
    let _ = &subst;
    asm_resolve(lines)
}

/// Second pass of `asm_program`: sizes, labels, then encoding.
fn asm_resolve(lines: Vec<(Option<String>, String)>) -> Asm {
    // Normalise "MNEMONIC a b" back to "MNEMONIC a, b".
    let norm = |l: &str| -> String {
        let (m, rest) = match l.split_once(' ') {
            Some((m, r)) => (m, r),
            None => return l.to_uppercase(),
        };
        let ops: Vec<&str> = rest.split(' ').filter(|s| !s.is_empty()).collect();
        format!("{} {}", m.to_uppercase(), ops.join(", ").replace(",,", ","))
    };
    let lines: Vec<(Option<String>, String)> = lines.into_iter().map(|(a, l)| (a, norm(&l))).collect();
    let words = |l: &str| -> usize {
        let m = l.split(' ').next().unwrap();
        if matches!(m, "LDS" | "STS" | "JMP" | "CALL") { 2 } else { 1 }
    };
    let mut labels = HashMap::new();
    let mut at = 0usize;
    for (label, l) in &lines {
        if let Some(name) = label {
            labels.insert(name.clone(), at * 2);
        }
        at += words(l);
    }
    let mut program = Vec::new();
    let mut nops = 0;
    for (_, l) in &lines {
        let here = program.len() * 2;
        let mut text = l.clone();
        if let Some((m, op)) = l.split_once(' ') {
            if matches!(m, "BRNE" | "RJMP") && labels.contains_key(op.trim()) {
                text = format!("{} {}", m, labels[op.trim()] as i64 - (here as i64 + 2));
            }
        }
        if text.starts_with("NOP") {
            nops += 1;
        }
        program.extend(asm_line(&text));
    }
    Asm { program, instruction_count: lines.len(), nop_count: nops, labels }
}

fn run_instructions(cpu: &mut Cpu, count: usize, on_break_throws: bool) {
    for _ in 0..count {
        if cpu.word(cpu.pc) == Some(0x9598) && on_break_throws {
            panic!("BREAK instruction encountered");
        }
        avr_instruction(cpu);
        cpu.tick();
    }
}

fn run_until(cpu: &mut Cpu, on_break_throws: bool, pred: impl Fn(&Cpu) -> bool) {
    for _ in 0..5000 {
        if cpu.word(cpu.pc) == Some(0x9598) && on_break_throws {
            panic!("BREAK instruction encountered");
        }
        if pred(cpu) {
            return;
        }
        avr_instruction(cpu);
        cpu.tick();
    }
    panic!("Test program ran for too long, check your predicate");
}

fn run_to_address(cpu: &mut Cpu, byte_addr: usize, on_break_throws: bool) {
    run_until(cpu, on_break_throws, |c| c.pc as usize * 2 == byte_addr);
}

fn run_to_break(cpu: &mut Cpu) {
    run_until(cpu, false, |c| c.word(c.pc) == Some(0x9598));
}

/// The calls `spyOn(port, 'timerOverridePin')` saw on `port`.
fn overrides_of(cpu: &Cpu, port: usize) -> Vec<(u8, PinOverride)> {
    cpu.p.overrides.iter().filter(|(p, _, _)| *p == port).map(|(_, pin, m)| (*pin, *m)).collect()
}

/// upstream: `should reflect the current port state when called inside a listener`
#[test]
fn gpio_pin_state_inside_a_listener() {
    let mut cpu = Cpu::new(vec![0; 1024], 8192);
    cpu.attach_port(PORT_B);
    assert_eq!(cpu.pin_state(PORT_B, 0), PinState::Input);
    cpu.write_data(0x24, 0x01);
    cpu.p.events.clear();
    cpu.write_data(0x25, 0x01);
    assert!(!cpu.p.events.is_empty());
    assert!(cpu.p.events.iter().all(|e| e.states[0] == PinState::High));
}

/// upstream: `should reflect the current port state when called inside a listener after DDR change`
#[test]
fn gpio_pin_state_inside_a_listener_after_ddr_change() {
    let mut cpu = Cpu::new(vec![0; 1024], 8192);
    cpu.attach_port(PORT_B);
    assert_eq!(cpu.pin_state(PORT_B, 0), PinState::Input);
    cpu.p.events.clear();
    cpu.write_data(0x24, 0x01);
    assert!(!cpu.p.events.is_empty());
    assert!(cpu.p.events.iter().all(|e| e.states[0] == PinState::Low));
}

/// gpio.spec.ts
pub mod gpio_spec {
    use super::*;
    #[allow(dead_code)]
    pub mod k {
        pub const SREG: i64 = 95;
        pub const PINB: i64 = 0x23;
        pub const DDRB: i64 = 0x24;
        pub const PORTB: i64 = 0x25;
        pub const PIND: i64 = 0x29;
        pub const DDRD: i64 = 0x2a;
        pub const PORTD: i64 = 0x2b;
        pub const EIFR: i64 = 0x3c;
        pub const EIMSK: i64 = 0x3d;
        pub const PCICR: i64 = 0x68;
        pub const EICRA: i64 = 0x69;
        pub const PCIFR: i64 = 0x3b;
        pub const PCMSK0: i64 = 0x6b;
        pub const INT0: i64 = 0;
        pub const ISC00: i64 = 0;
        pub const ISC01: i64 = 1;
        pub const PCIE0: i64 = 0;
        pub const PCINT3: i64 = 3;
        pub const PB0: i64 = 0;
        pub const PB1: i64 = 1;
        pub const PB3: i64 = 3;
        pub const PB4: i64 = 4;
        pub const PD2: i64 = 2;
        pub const PC_INT_INT0: i64 = 2;
        pub const PC_INT_PCINT0: i64 = 6;
    }
    use k::*;

    /// upstream: `should invoke the listeners when the port is written to`
    #[test]
    fn should_invoke_the_listeners_when_the_port_is_written_to() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0x0f) as u8);
        cpu.p.events.clear();
    cpu.p.listening = true;
        cpu.write_data((PORTB) as i32, (0x55) as u8);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x55) as i64 && e.prev as i64 == (0) as i64));
        assert_eq!(((cpu.data[(0x23) as usize] as i64)) as i64, (0x5) as i64);  // PINB should return port value
    }

    /// upstream: `should invoke the listeners when DDR changes (issue #28)`
    #[test]
    fn should_invoke_the_listeners_when_ddr_changes_issue_28() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((PORTB) as i32, (0x55) as u8);
        cpu.p.events.clear();
    cpu.p.listening = true;
        cpu.write_data((DDRB) as i32, (0xf0) as u8);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x55) as i64 && e.prev as i64 == (0x55) as i64));
    }

    /// upstream: `should invoke the listeners when pullup register enabled (issue #62)`
    #[test]
    fn should_invoke_the_listeners_when_pullup_register_enabled_issue_62() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.p.events.clear();
    cpu.p.listening = true;
        cpu.write_data((PORTB) as i32, (0x55) as u8);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x55) as i64 && e.prev as i64 == (0) as i64));
    }

    /// upstream: `should toggle the pin when writing to the PIN register`
    #[test]
    fn should_toggle_the_pin_when_writing_to_the_pin_register() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.p.events.clear();
    cpu.p.listening = true;
        cpu.write_data((DDRB) as i32, (0x0f) as u8);
        cpu.write_data((PORTB) as i32, (0x55) as u8);
        cpu.write_data((PINB) as i32, (0x01) as u8);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x54) as i64 && e.prev as i64 == (0x55) as i64));
        assert_eq!(((cpu.data[(PINB) as usize] as i64)) as i64, (0x4) as i64);  // PINB should return port value
    }

    /// upstream: `should only affect one pin when writing to PIN using SBI (issue #103)`
    #[test]
    fn should_only_affect_one_pin_when_writing_to_pin_using_sbi_issue_103() {
        let asm = asm_program(&format!(r#"
    ; register addresses
    _REPLACE DDRD, {}
    _REPLACE PIND, {}
    _REPLACE PORTD, {}

    ; Setup
    ldi r24, 0x48
    out DDRD, r24
    out PORTD, r24

    ; Now toggle pin 6 with SBI
    sbi PIND, 6

    break"#, DDRD - 0x20, PIND - 0x20, PORTD - 0x20));
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_port(PORT_D);
        let on_break_throws = true;
        cpu.p.events.clear();
    cpu.p.listening = true;
        // Setup: pins 6, 3 are output, set to HIGH
        run_instructions(&mut cpu, (3) as usize, on_break_throws);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x48) as i64 && e.prev as i64 == (0x0) as i64));
        assert_eq!(((cpu.data[(PORTD) as usize] as i64)) as i64, (0x48) as i64);
        cpu.p.events.clear();
        // Now we toggle pin 6
        run_instructions(&mut cpu, (1) as usize, on_break_throws);
        assert!(cpu.p.events.iter().any(|e| e.value as i64 == (0x08) as i64 && e.prev as i64 == (0x48) as i64));
        assert_eq!(((cpu.data[(PORTD) as usize] as i64)) as i64, (0x8) as i64);
    }

    /// upstream: `should update the PIN register on output compare (OCR) match (issue #102)`
    #[test]
    fn should_update_the_pin_register_on_output_compare_ocr_match_issue_102() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (1 << 1) as u8);
        cpu.timer_override_pin(PORT_B, (1) as u8, PinOverride::Set);
        assert_eq!(((cpu.pin_state(PORT_B, (1) as u8) as i64)) as i64, ((PinState::High as i64)) as i64);
        assert_eq!(((cpu.data[(PINB) as usize] as i64)) as i64, (1 << 1) as i64);
        cpu.timer_override_pin(PORT_B, (1) as u8, PinOverride::Clear);
        assert_eq!(((cpu.pin_state(PORT_B, (1) as u8) as i64)) as i64, ((PinState::Low as i64)) as i64);
        assert_eq!(((cpu.data[(PINB) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should remove the given listener`
    #[test]
    fn should_remove_the_given_listener() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.p.events.clear();
    cpu.p.listening = true;
        cpu.write_data((DDRB) as i32, (0x0f) as u8);
        cpu.p.listening = false;
        cpu.write_data((PORTB) as i32, (0x99) as u8);
        assert_eq!(cpu.p.events.len(), 1);
    }

    /// upstream: `should return PinState.High when the pin set to output and HIGH`
    #[test]
    fn should_return_pinstate_high_when_the_pin_set_to_output_and_high() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0x1) as u8);
        cpu.write_data((PORTB) as i32, (0x1) as u8);
        assert_eq!(((cpu.pin_state(PORT_B, (PB0) as u8) as i64)) as i64, ((PinState::High as i64)) as i64);
    }

    /// upstream: `should return PinState.Low when the pin set to output and LOW`
    #[test]
    fn should_return_pinstate_low_when_the_pin_set_to_output_and_low() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0x8) as u8);
        cpu.write_data((PORTB) as i32, (0xf7) as u8);
        assert_eq!(((cpu.pin_state(PORT_B, (PB3) as u8) as i64)) as i64, ((PinState::Low as i64)) as i64);
    }

    /// upstream: `should return PinState.Input by default (reset state)`
    #[test]
    fn should_return_pinstate_input_by_default_reset_state() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        assert_eq!(((cpu.pin_state(PORT_B, (PB1) as u8) as i64)) as i64, ((PinState::Input as i64)) as i64);
    }

    /// upstream: `should return PinState.InputPullUp when the pin is set to input with pullup`
    #[test]
    fn should_return_pinstate_inputpullup_when_the_pin_is_set_to_input_with_pullup() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0) as u8);
        cpu.write_data((PORTB) as i32, (0x2) as u8);
        assert_eq!(((cpu.pin_state(PORT_B, (PB1) as u8) as i64)) as i64, ((PinState::InputPullUp as i64)) as i64);
    }

    /// upstream: `should set the value of the given pin`
    #[test]
    fn should_set_the_value_of_the_given_pin() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0) as u8);
        cpu.set_pin(PORT_B, (PB4) as u8, true);
        assert_eq!(((cpu.data[(0x23) as usize] as i64)) as i64, (0x10) as i64);
        cpu.set_pin(PORT_B, (PB4) as u8, false);
        assert_eq!(((cpu.data[(0x23) as usize] as i64)) as i64, (0x0) as i64);
    }

    /// upstream: `should only update PIN register when pin in Input mode`
    #[test]
    fn should_only_update_pin_register_when_pin_in_input_mode() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((DDRB) as i32, (0x10) as u8);
        cpu.write_data((PORTB) as i32, (0x0) as u8);
        cpu.set_pin(PORT_B, (PB4) as u8, true);
        assert_eq!(((cpu.data[(PINB) as usize] as i64)) as i64, (0x0) as i64);
        cpu.write_data((DDRB) as i32, (0x0) as u8);
        assert_eq!(((cpu.data[(PINB) as usize] as i64)) as i64, (0x10) as i64);
    }

    /// upstream: `should generate INT0 interrupt on rising edge`
    #[test]
    fn should_generate_int0_interrupt_on_rising_edge() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_D);
        cpu.write_data((EIMSK) as i32, (1 << INT0) as u8);
        cpu.write_data((EICRA) as i32, ((1 << ISC01) | (1 << ISC00)) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, true);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I------- (enable interrupts)
        cpu.tick();
        assert_eq!(((cpu.pc as i64)) as i64, (PC_INT_INT0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, false);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should generate INT0 interrupt on falling edge`
    #[test]
    fn should_generate_int0_interrupt_on_falling_edge() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_D);
        cpu.write_data((EIMSK) as i32, (1 << INT0) as u8);
        cpu.write_data((EICRA) as i32, (1 << ISC01) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, true);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, false);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I------- (enable interrupts)
        cpu.tick();
        assert_eq!(((cpu.pc as i64)) as i64, (PC_INT_INT0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should generate INT0 interrupt on level change`
    #[test]
    fn should_generate_int0_interrupt_on_level_change() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_D);
        cpu.write_data((EIMSK) as i32, (1 << INT0) as u8);
        cpu.write_data((EICRA) as i32, (1 << ISC00) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, true);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        cpu.write_data((EIFR) as i32, (1 << INT0) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, false);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
    }

    /// upstream: `should a sticky INT0 interrupt while the pin level is low`
    #[test]
    fn should_a_sticky_int0_interrupt_while_the_pin_level_is_low() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_D);
        cpu.write_data((EIMSK) as i32, (1 << INT0) as u8);
        cpu.write_data((EICRA) as i32, (0) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, true);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (PD2) as u8, false);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        // This is a sticky interrupt, verify we can't clear the flag:
        cpu.write_data((EIFR) as i32, (1 << INT0) as u8);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I------- (enable interrupts)
        cpu.tick();
        assert_eq!(((cpu.pc as i64)) as i64, (PC_INT_INT0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
        // Flag shouldn't be cleared, as the interrupt is sticky
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (1 << INT0) as i64);
        // But it will be cleared as soon as the pin goes high.
        cpu.set_pin(PORT_D, (PD2) as u8, true);
        assert_eq!(((cpu.data[(EIFR) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should generate a pin change interrupt when PB3 (PCINT3) goes high`
    #[test]
    fn should_generate_a_pin_change_interrupt_when_pb3_pcint3_goes_high() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((PCICR) as i32, (1 << PCIE0) as u8);
        cpu.write_data((PCMSK0) as i32, (1 << PCINT3) as u8);
        cpu.set_pin(PORT_B, (PB3) as u8, true);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (1 << PCIE0) as i64);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I-------
        cpu.tick();
        assert_eq!(((cpu.pc as i64)) as i64, (PC_INT_PCINT0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should generate a pin change interrupt when PB3 (PCINT3) goes low`
    #[test]
    fn should_generate_a_pin_change_interrupt_when_pb3_pcint3_goes_low() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.set_pin(PORT_B, (PB3) as u8, true);
        cpu.write_data((PCICR) as i32, (1 << PCIE0) as u8);
        cpu.write_data((PCMSK0) as i32, (1 << PCINT3) as u8);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_B, (PB3) as u8, false);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (1 << PCIE0) as i64);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I-------
        cpu.tick();
        assert_eq!(((cpu.pc as i64)) as i64, (PC_INT_PCINT0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (0) as i64);
    }

    /// upstream: `should clear the interrupt flag when writing to PCIFR`
    #[test]
    fn should_clear_the_interrupt_flag_when_writing_to_pcifr() {
        let mut cpu = Cpu::new(vec![0; 1024], 8192);
        cpu.attach_port(PORT_B);
        cpu.write_data((PCICR) as i32, (1 << PCIE0) as u8);
        cpu.write_data((PCMSK0) as i32, (1 << PCINT3) as u8);
        cpu.set_pin(PORT_B, (PB3) as u8, true);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (1 << PCIE0) as i64);
        cpu.write_data((PCIFR) as i32, (1 << PCIE0) as u8);
        assert_eq!(((cpu.data[(PCIFR) as usize] as i64)) as i64, (0) as i64);
    }

}

/// adc.spec.ts
pub mod adc_spec {
    use super::*;
    #[allow(dead_code)]
    pub mod k {
        pub const R16: i64 = 16;
        pub const R17: i64 = 17;
        pub const ADMUX: i64 = 0x7c;
        pub const REFS0: i64 = 1 << 6;
        pub const ADCSRA: i64 = 0x7a;
        pub const ADEN: i64 = 1 << 7;
        pub const ADSC: i64 = 1 << 6;
        pub const ADPS0: i64 = 1 << 0;
        pub const ADPS1: i64 = 1 << 1;
        pub const ADPS2: i64 = 1 << 2;
        pub const ADCH: i64 = 0x79;
        pub const ADCL: i64 = 0x78;
    }
    use k::*;

    /// upstream: `should successfuly perform an ADC conversion`
    #[test]
    fn should_successfuly_perform_an_adc_conversion() {
        let asm = asm_program(&format!(r#"
    ; register addresses
    _REPLACE ADMUX, {}
    _REPLACE ADCSRA, {}
    _REPLACE ADCH, {}
    _REPLACE ADCL, {}

    ; Configure mux - channel 0, reference: AVCC with external capacitor at AREF pin
    ldi r24, {}
    sts ADMUX, r24

    ; Start conversion with 128 prescaler
    ldi r24, {}
    sts ADCSRA, r24

    ; Wait until conversion is complete
  waitComplete:
    lds r24, {}
    andi r24, {}
    brne waitComplete

    ; Read the result
    lds r16, {}
    lds r17, {}

    break"#, ADMUX, ADCSRA, ADCH, ADCL, REFS0, ADEN | ADSC | ADPS0 | ADPS1 | ADPS2, ADCSRA, ADSC, ADCL, ADCH));
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_adc();
        let on_break_throws = true;
        cpu.p.log_overrides = true; // spy on onADCRead: the port records each read in cpu.p.adc_reads
        cpu.set_adc_channel(0, 2.56);
        // Setup
        run_instructions(&mut cpu, (4) as usize, on_break_throws);
        assert!(cpu.p.adc_reads.contains(&MuxInput::SingleEnded(0)));
        // Run the "waitComplete" loop for a few cycles
        run_instructions(&mut cpu, (12) as usize, on_break_throws);
        cpu.cycles += (128 * 25) as u64;
        cpu.tick();
        // Now read the result
        run_instructions(&mut cpu, (5) as usize, on_break_throws);
        let low = cpu.data[(R16) as usize] as i64;
        let high = cpu.data[(R17) as usize] as i64;
        assert_eq!(((high << 8) | low) as i64, (524) as i64);  // 2.56 volts - see above
    }

    /// upstream: `should read 0 when the ADC peripheral is not enabled`
    #[test]
    fn should_read_0_when_the_adc_peripheral_is_not_enabled() {
        // This behavior was verified on real hardware, using the following test program:
        // https://wokwi.com/arduino/projects/309156042450666050
        // Thanks Oscar Oomens for spotting this!
        let asm = asm_program(&format!(r#"
    ; register addresses
    _REPLACE ADMUX, {}
    _REPLACE ADCSRA, {}
    _REPLACE ADCH, {}
    _REPLACE ADCL, {}

    ; Load some initial value into r16/r17 to make sure we actually read 0 later
    ldi r16, 0xff
    ldi r17, 0xff

    ; Configure mux - channel 0, reference: AVCC with external capacitor at AREF pin
    ldi r24, {}
    sts ADMUX, r24

    ; Start conversion with 128 prescaler, but without enabling the ADC
    ldi r24, {}
    sts ADCSRA, r24

    ; Wait until conversion is complete
  waitComplete:
    lds r24, {}
    andi r24, {}
    brne waitComplete

    ; Read the result
    lds r16, {}
    lds r17, {}

    break"#, ADMUX, ADCSRA, ADCH, ADCL, REFS0, ADSC | ADPS0 | ADPS1 | ADPS2, ADCSRA, ADSC, ADCL, ADCH));
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_adc();
        let on_break_throws = false;
        cpu.p.log_overrides = true; // spy on onADCRead: the port records each read in cpu.p.adc_reads
        cpu.set_adc_channel(0, 2.56);
        // Setup
        run_instructions(&mut cpu, (6) as usize, on_break_throws);
        assert!(cpu.p.adc_reads.is_empty());
        // Run the "waitComplete" loop for a few cycles
        run_instructions(&mut cpu, (12) as usize, on_break_throws);
        cpu.cycles += (128 * 25) as u64;
        cpu.tick();
        // Now read the result
        run_to_break(&mut cpu);
        let low = cpu.data[(R16) as usize] as i64;
        let high = cpu.data[(R17) as usize] as i64;
        assert_eq!(((high << 8) | low) as i64, (0) as i64);  // We should read 0 since the ADC hasn't been enabled
    }

}

/// timer.spec.ts
pub mod timer_spec {
    use super::*;
    #[allow(dead_code)]
    pub mod k {
        pub const R1: i64 = 1;
        pub const R17: i64 = 17;
        pub const R18: i64 = 18;
        pub const R19: i64 = 19;
        pub const R20: i64 = 20;
        pub const R21: i64 = 21;
        pub const R22: i64 = 22;
        pub const SREG: i64 = 95;
        pub const TIFR0: i64 = 0x35;
        pub const TCCR0A: i64 = 0x44;
        pub const TCCR0B: i64 = 0x45;
        pub const TCNT0: i64 = 0x46;
        pub const OCR0A: i64 = 0x47;
        pub const OCR0B: i64 = 0x48;
        pub const TIMSK0: i64 = 0x6e;
        pub const TIMSK1: i64 = 0x6f;
        pub const TIFR1: i64 = 0x36;
        pub const TCCR1A: i64 = 0x80;
        pub const TCCR1B: i64 = 0x81;
        pub const TCCR1C: i64 = 0x82;
        pub const TCNT1: i64 = 0x84;
        pub const TCNT1H: i64 = 0x85;
        pub const ICR1: i64 = 0x86;
        pub const ICR1H: i64 = 0x87;
        pub const OCR1A: i64 = 0x88;
        pub const OCR1AH: i64 = 0x89;
        pub const OCR1B: i64 = 0x8a;
        pub const OCR1C: i64 = 0x8c;
        pub const OCR1CH: i64 = 0x8d;
        pub const TCCR2B: i64 = 0xb1;
        pub const TCNT2: i64 = 0xb2;
        pub const TOV0: i64 = 1;
        pub const TOV1: i64 = 1;
        pub const OCIE0A: i64 = 2;
        pub const OCIE0B: i64 = 4;
        pub const TOIE0: i64 = 1;
        pub const OCF0A: i64 = 2;
        pub const OCF0B: i64 = 4;
        pub const OCF1A: i64 = 1 << 1;
        pub const OCF1B: i64 = 1 << 2;
        pub const OCF1C: i64 = 1 << 3;
        pub const WGM00: i64 = 1;
        pub const WGM10: i64 = 1;
        pub const WGM01: i64 = 2;
        pub const WGM11: i64 = 2;
        pub const WGM12: i64 = 8;
        pub const WGM13: i64 = 16;
        pub const CS00: i64 = 1;
        pub const CS01: i64 = 2;
        pub const CS02: i64 = 4;
        pub const CS10: i64 = 1;
        pub const CS21: i64 = 2;
        pub const CS22: i64 = 4;
        pub const COM0B1: i64 = 1 << 5;
        pub const COM1C0: i64 = 1 << 2;
        pub const FOC0B: i64 = 1 << 6;
        pub const FOC1C: i64 = 1 << 5;
        pub const T0: i64 = 4;
    }
    use k::*;

    /// upstream: `should update timer every tick when prescaler is 1`
    #[test]
    fn should_update_timer_every_tick_when_prescaler_is_1() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (1) as i64);
    }

    /// upstream: `should update timer every 64 ticks when prescaler is 3`
    #[test]
    fn should_update_timer_every_64_ticks_when_prescaler_is_3() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0B) as i32, (CS01 | CS00) as u8);  // Set prescaler to 64
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (1 + 64) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (1) as i64);
    }

    /// upstream: `should not update timer if it has been disabled`
    #[test]
    fn should_not_update_timer_if_it_has_been_disabled() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0B) as i32, (0) as u8);  // No prescaler (timer disabled)
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (100000) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0) as i64);  // TCNT should stay 0
    }

    /// upstream: `should set the TOV flag when timer wraps above TOP value`
    #[test]
    fn should_set_the_tov_flag_when_timer_wraps_above_top_value() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xff) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);
        cpu.cycles += 1;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
    }

    /// upstream: `should set the TOV if timer overflows past TOP without reaching TOP`
    #[test]
    fn should_set_the_tov_if_timer_overflows_past_top_without_reaching_top() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xfe) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        cpu.cycles += (4) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0x2) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
    }

    /// upstream: `should clear the TOV flag when writing 1 to the TOV bit, and not trigger the interrupt`
    #[test]
    fn should_clear_the_tov_flag_when_writing_1_to_the_tov_bit_and_not_trigger_the_interrupt() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
        cpu.write_data((TIFR0) as i32, (TOV0) as u8);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);
    }

    /// upstream: `should set TOV if timer overflows in FAST PWM mode`
    #[test]
    fn should_set_tov_if_timer_overflows_in_fast_pwm_mode() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.write_data((OCR0A) as i32, (0x7f) as u8);
        cpu.write_data((TCCR0A) as i32, (WGM01 | WGM00) as u8);  // WGM: Fast PWM
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
    }

    /// upstream: `should generate an overflow interrupt if timer overflows and interrupts enabled`
    #[test]
    fn should_generate_an_overflow_interrupt_if_timer_overflows_and_interrupts_enabled() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.write_data((TIMSK0) as i32, (TOIE0) as u8);
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I-------
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (2) as i64);  // TCNT should be 2 (one tick above + 2 cycles for interrupt)
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0x20) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (4) as i64);
    }

    /// upstream: `should not generate an overflow interrupt when global interrupts disabled`
    #[test]
    fn should_not_generate_an_overflow_interrupt_when_global_interrupts_disabled() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.data[(TIMSK0) as usize] = (TOIE0) as u8;
        cpu.data[(SREG) as usize] = (0x0) as u8;  // SREG: --------
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should not generate an overflow interrupt when TOIE0 is clear`
    #[test]
    fn should_not_generate_an_overflow_interrupt_when_toie0_is_clear() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.data[(TIMSK0) as usize] = (0) as u8;
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I-------
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should set OCF0A/B flags when OCRA/B == 0 and the timer equals to OCRA (issue #74)`
    #[test]
    fn should_set_ocf0a_b_flags_when_ocra_b_0_and_the_timer_equals_to_ocra_issue_74() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xff) as u8);
        cpu.write_data((OCR0A) as i32, (0x0) as u8);
        cpu.write_data((OCR0B) as i32, (0x0) as u8);
        cpu.write_data((TCCR0A) as i32, (0x0) as u8);  // WGM: Normal
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & (OCF0A | OCF0B)) as i64, (OCF0A | OCF0B) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should set the OCF1A flag when OCR1A == 120 and the timer overflowed past 120 in WGM mode 15 (issue #94)`
    #[test]
    fn should_set_the_ocf1a_flag_when_ocr1a_120_and_the_timer_overflowed_past_120_in_wgm_mode_15_issue_94() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1) as i32, (118) as u8);
        cpu.write_data((OCR1A) as i32, (120) as u8);
        cpu.write_data((OCR1B) as i32, (4) as u8);  // To avoid getting the OCF1B flag set
        cpu.write_data((TCCR1A) as i32, (WGM10 | WGM11) as u8);  // WGM: Fast PWM
        cpu.write_data((TCCR1B) as i32, (WGM12 | WGM13 | CS10) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (5) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT1) as i32) as i64)) as i64, (1) as i64);
        assert_eq!(((cpu.data[(TIFR1) as usize] as i64) & (OCF1A | OCF1B)) as i64, (OCF1A) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (5) as i64);
    }

    /// upstream: `should set OCF0A flag when timer equals OCRA`
    #[test]
    fn should_set_ocf0a_flag_when_timer_equals_ocra() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x10) as u8);
        cpu.write_data((OCR0A) as i32, (0x11) as u8);
        cpu.write_data((TCCR0A) as i32, (0x0) as u8);  // WGM: Normal
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64)) as i64, (OCF0A) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should reset the counter in CTC mode if it equals to OCRA`
    #[test]
    fn should_reset_the_counter_in_ctc_mode_if_it_equals_to_ocra() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x10) as u8);
        cpu.write_data((OCR0A) as i32, (0x11) as u8);
        cpu.write_data((TCCR0A) as i32, (WGM01) as u8);  // WGM: CTC
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (3) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (3) as i64);
    }

    /// upstream: `should not set the TOV bit when TOP < MAX in CTC mode (issue #75)`
    #[test]
    fn should_not_set_the_tov_bit_when_top_max_in_ctc_mode_issue_75() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x1e) as u8);
        cpu.write_data((OCR0A) as i32, (0x1f) as u8);
        cpu.write_data((TCCR0A) as i32, (WGM01) as u8);  // WGM: CTC
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles += 1;
        cpu.tick();
        cpu.cycles += 1;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);  // TOV0 clear
    }

    /// upstream: `should set the TOV bit when TOP == MAX in CTC mode (issue #75)`
    #[test]
    fn should_set_the_tov_bit_when_top_max_in_ctc_mode_issue_75() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xfe) as u8);
        cpu.write_data((OCR0A) as i32, (0xff) as u8);
        cpu.write_data((TCCR0A) as i32, (WGM01) as u8);  // WGM: CTC
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles += 1;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xff) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);  // TOV clear
        cpu.cycles += 1;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);  // TOV set
    }

    /// upstream: `should not set the TOV bit twice on overflow (issue #80)`
    #[test]
    fn should_not_set_the_tov_bit_twice_on_overflow_issue_80() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0xfe) as u8);
        cpu.write_data((OCR0A) as i32, (0xff) as u8);
        cpu.write_data((TCCR0A) as i32, (WGM01) as u8);  // WGM: CTC
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles += 1;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xff) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (0) as i64);  // TOV clear
        cpu.cycles += 1;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);  // TOV set
    }

    /// upstream: `should set OCF0B flag when timer equals OCRB`
    #[test]
    fn should_set_ocf0b_flag_when_timer_equals_ocrb() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x10) as u8);
        cpu.write_data((OCR0B) as i32, (0x11) as u8);
        cpu.write_data((TCCR0A) as i32, (0x0) as u8);  // WGM: (Normal)
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64)) as i64, (OCF0B) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should generate Timer Compare A interrupt when TCNT0 == TCNTA`
    #[test]
    fn should_generate_timer_compare_a_interrupt_when_tcnt0_tcnta() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x20) as u8);
        cpu.write_data((OCR0A) as i32, (0x21) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.write_data((TIMSK0) as i32, (OCIE0A) as u8);
        cpu.write_data((95) as i32, (0x80) as u8);  // SREG: I-------
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0x23) as i64);  // TCNT should be 0x23 (one tick above + 2 cycles for interrupt)
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & OCF0A) as i64, (0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0x1c) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (4) as i64);
    }

    /// upstream: `should not generate Timer Compare A interrupt when OCIEA is disabled`
    #[test]
    fn should_not_generate_timer_compare_a_interrupt_when_ociea_is_disabled() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x20) as u8);
        cpu.write_data((OCR0A) as i32, (0x21) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.write_data((TIMSK0) as i32, (0) as u8);
        cpu.write_data((95) as i32, (0x80) as u8);  // SREG: I-------
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0x21) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should generate Timer Compare B interrupt when TCNT0 == TCNTB`
    #[test]
    fn should_generate_timer_compare_b_interrupt_when_tcnt0_tcntb() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCNT0) as i32, (0x20) as u8);
        cpu.write_data((OCR0B) as i32, (0x21) as u8);
        cpu.write_data((TCCR0B) as i32, (CS00) as u8);  // Set prescaler to 1
        cpu.write_data((TIMSK0) as i32, (OCIE0B) as u8);
        cpu.write_data((95) as i32, (0x80) as u8);  // SREG: I-------
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        let tcnt = cpu.read_data((TCNT0) as i32) as i64;
        assert_eq!((tcnt) as i64, (0x23) as i64);  // TCNT should be 0x23 (0x23 + 2 cycles for interrupt)
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & OCF0B) as i64, (0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0x1e) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (4) as i64);
    }

    /// upstream: `should not increment TCNT on the same cycle of TCNT write (issue #36)`
    #[test]
    fn should_not_increment_tcnt_on_the_same_cycle_of_tcnt_write_issue_36() {
        // At the end of this short program, R17 should contain 0x31. Verified against
        // a physical ATmega328p.
        let asm = asm_program(r#"
      LDI r16, 0x1    ; TCCR0B = 1 << CS00;
      OUT 0x25, r16
      LDI r16, 0x30   ; TCNT0 <- 0x30
      OUT 0x26, r16
      NOP
      IN r17, 0x26    ; r17 <- TCNT"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.data[(R17) as usize] as i64)) as i64, (0x31) as i64);
    }

    /// upstream: `timer2 should count every 256 ticks when prescaler is 6 (issue #5)`
    #[test]
    fn timer2_should_count_every_256_ticks_when_prescaler_is_6_issue_5() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(2);
        cpu.write_data((TCCR2B) as i32, (CS22 | CS21) as u8);  // Set prescaler to 256
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (1 + 511) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT2) as i32) as i64)) as i64, (1) as i64);
        cpu.cycles = (1 + 512) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT2) as i32) as i64)) as i64, (2) as i64);
    }

    /// upstream: `should update TCNT as it is being read by a 2-cycle instruction (issue #40)`
    #[test]
    fn should_update_tcnt_as_it_is_being_read_by_a_2_cycle_instruction_issue_40() {
        let asm = asm_program(r#"
      LDI r16, 0x1      ; TCCR0B = 1 << CS00
      OUT 0x25, r16
      LDI r16, 0x0      ; TCNT0 <- 0
      OUT 0x26, r16
      NOP
      LDS r1, 0x46      ; r1 <- TCNT0 (2 cycles)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.data[(R1) as usize] as i64)) as i64, (2) as i64);
    }

    /// upstream: `should not start counting before the prescaler is first set (issue #41)`
    #[test]
    fn should_not_start_counting_before_the_prescaler_is_first_set_issue_41() {
        let asm = asm_program(r#"
      NOP
      NOP
      NOP
      NOP
      LDI r16, 0x1    ; TCCR2B = 1 << CS20;
      STS 0xb1, r16   ; Should start counting after this line
      NOP
      LDS r17, 0xb2   ; TCNT should equal 2 at this point"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(2);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (2) as i64);
    }

    /// upstream: `should not keep counting for one more instruction when the timer is disabled (issue #72)`
    #[test]
    fn should_not_keep_counting_for_one_more_instruction_when_the_timer_is_disabled_issue_72() {
        let asm = asm_program(r#"
      EOR r1, r1      ; r1 = 0;
      LDI r16, 0x1    ; TCCR2B = 1 << CS20;
      STS 0xb1, r16   ; Should start counting after this instruction,
      STS 0xb1, r1    ; and stop counting *after* this one.
      NOP
      LDS r17, 0xb2   ; TCNT2 should equal 2 at this point (not counting the NOP)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(2);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (2) as i64);
    }

    /// upstream: `should clear OC0B pin when writing 1 to FOC0B`
    #[test]
    fn should_clear_oc0b_pin_when_writing_1_to_foc0b() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0A) as i32, (COM0B1) as u8);
        // Listen to Port B's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        cpu.write_data((TCCR0B) as i32, (FOC0B) as u8);
        assert!(overrides_of(&cpu, PORT_D).contains(&((5) as u8, PinOverride::Clear)));
    }

    /// upstream: `should set OC0A on Compare Match, clear on Bottom (issue #78)`
    #[test]
    fn should_set_oc0a_on_compare_match_clear_on_bottom_issue_78() {
        let asm = asm_program(r#"
        LDI r16, 0xfc   ; TCNT0 = 0xfc;
        OUT 0x26, r16
        LDI r16, 0xfe   ; OCR0A = 0xfe;
        OUT 0x27, r16  
        ; WGM: Fast PWM, enable OC0A mode 3 (set on Compare Match, clear on Bottom)
        LDI r16, 0xc3   ; TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
        OUT 0x24, r16
        LDI r16, 0x1    ; TCCR0B = 1 << CS00;
        OUT 0x25, r16

        NOP             ; TCNT is now 0xfd
      beforeMatch: 
        NOP             ; TCNT is now 0xfe (Compare Match)
      afterMatch:
        NOP             ; TCNT is now 0xff
      beforeBottom:     
        NOP             ; TCNT is now 0x00 (BOTTOM)
      afterBottom:
        NOP"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        run_to_address(&mut cpu, asm.labels["beforeMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfd) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["afterMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Set)));
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["beforeBottom"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xff) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 0);
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["afterBottom"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0x0) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Clear)));
    }

    /// upstream: `should toggle OC0A on Compare Match when COM0An = 1 (issue #78)`
    #[test]
    fn should_toggle_oc0a_on_compare_match_when_com0an_1_issue_78() {
        let asm = asm_program(r#"
        LDI r16, 0xfc   ; TCNT0 = 0xfc;
        OUT 0x26, r16
        LDI r16, 0xfe   ; OCR0A = 0xfe;
        OUT 0x27, r16  
        ; WGM: Fast PWM, enable OC0A mode 1 (Toggle)
        LDI r16, 0x43   ; TCCR0A = (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
        OUT 0x24, r16
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16

        NOP             ; TCNT is now 0xfd
      beforeMatch: 
        NOP             ; TCNT is now 0xfe (Compare Match, TOP)
      afterMatch:
        NOP             ; TCNT is now 0
      afterOverflow:
        NOP"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        run_to_address(&mut cpu, asm.labels["beforeMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfd) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["afterMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Toggle)));
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["afterOverflow"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 0);
    }

    /// upstream: `should leave OC0A disconnected when COM0An = 1 and WGM02 = 0 (issue #78)`
    #[test]
    fn should_leave_oc0a_disconnected_when_com0an_1_and_wgm02_0_issue_78() {
        let asm = asm_program(r#"
        LDI r16, 0xfc   ; TCNT0 = 0xfc;
        OUT 0x26, r16
        LDI r16, 0xfe   ; OCR0A = 0xfe;
        OUT 0x27, r16  
        ; WGM: Fast PWM mode 7, enable OC0A mode 1 (Toggle)
        LDI r16, 0x43   ; TCCR0A = (1 << COM0A0) | (1 << WGM01) | (1 << WGM00);
        OUT 0x24, r16
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16
        
      beforeClearWGM02:
        LDI r16, 0x01   ; TCCR0B = (1 << CS00);
        OUT 0x25, r16

      afterClearWGM02:
        NOP"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        // First, run with the bit set and assert that the Pin Override was enabled (OC0A connected)
        run_to_address(&mut cpu, asm.labels["beforeClearWGM02"], on_break_throws);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        // Now clear WGM02 and observe that Pin Override was disabled (OC0A disconnected)
        run_to_address(&mut cpu, asm.labels["afterClearWGM02"], on_break_throws);
        assert_eq!(overrides_of(&cpu, PORT_D).len(), 1);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::None)));
        cpu.p.overrides.clear();
    }

    /// upstream: `should count up to TOP, down to 0, and then set TOV flag`
    #[test]
    fn should_count_up_to_top_down_to_0_and_then_set_tov_flag() {
        let asm = asm_program(r#"
        LDI r16, 0x3   ; OCR0A = 0x3;   // <- TOP value
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to PWM, Phase Correct, top OCR0A
        LDI r16, 0x1   ; TCCR0A = 1 << WGM00;
        OUT 0x24, r16  
        LDI r16, 0x9   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0x2   ; TCNT0 = 0x2;
        OUT 0x26, r16

        IN r17, 0x26   ; TCNT0 will be 2
        IN r18, 0x26   ; TCNT0 will be 3
        IN r19, 0x26   ; TCNT0 will be 2
        IN r20, 0x26   ; TCNT0 will be 1
        IN r21, 0x26   ; TCNT0 will be 0
        IN r22, 0x26   ; TCNT0 will be 1 (end of test)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.read_data((R18) as i32) as i64)) as i64, (3) as i64);
        assert_eq!(((cpu.read_data((R19) as i32) as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.read_data((R20) as i32) as i64)) as i64, (1) as i64);
        assert_eq!(((cpu.read_data((R21) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.read_data((R22) as i32) as i64)) as i64, (1) as i64);
        assert_eq!(((cpu.data[(TIFR0) as usize] as i64) & TOV0) as i64, (TOV0) as i64);
    }

    /// upstream: `should clear OC0A when TCNT0=OCR0A and counting up`
    #[test]
    fn should_clear_oc0a_when_tcnt0_ocr0a_and_counting_up() {
        let asm = asm_program(r#"
        LDI r16, 0xfe   ; OCR0A = 0xfe;   // <- TOP value
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to PWM, Phase Correct
        LDI r16, 0x81   ; TCCR0A = (1 << COM0A1) | (1 << WGM00);
        OUT 0x24, r16  
        LDI r16, 0x1   ; TCCR0B = (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0xfd   ; TCNT0 = 0xfd;
        OUT 0x26, r16  

        NOP   ; TCNT0 will be 0xfe
        NOP   ; TCNT0 will be 0xff
        NOP   ; TCNT0 will be 0xfe again (end of test)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let nop_count = asm.nop_count;
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count - nop_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfd) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        run_instructions(&mut cpu, (1) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Clear)));
        cpu.p.overrides.clear();
        run_instructions(&mut cpu, (1) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xff) as i64);
        assert!(overrides_of(&cpu, PORT_D).is_empty());
        run_instructions(&mut cpu, (1) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Set)));
    }

    /// upstream: `should toggle OC0A when TCNT0=OCR0A and COM0An=1 (issue #78)`
    #[test]
    fn should_toggle_oc0a_when_tcnt0_ocr0a_and_com0an_1_issue_78() {
        let asm = asm_program(r#"
        LDI r16, 0xfe   ; OCR0A = 0xfe;   // <- TOP value
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to PWM, Phase Correct (mode 5)
        LDI r16, 0x41   ; TCCR0A = (1 << COM0A0) | (1 << WGM00);
        OUT 0x24, r16  
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0xfd   ; TCNT0 = 0xfd;
        OUT 0x26, r16  

      beforeMatch:
        NOP             ; TCNT0 will be 0xfe
      afterMatch:
        NOP"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        run_to_address(&mut cpu, asm.labels["beforeMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfd) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        run_to_address(&mut cpu, asm.labels["afterMatch"], on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0xfe) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Toggle)));
        cpu.p.overrides.clear();
    }

    /// upstream: `should leave OC0A disconnected TCNT0=OCR0A and COM0An=1 in WGM mode 1 (issue #78)`
    #[test]
    fn should_leave_oc0a_disconnected_tcnt0_ocr0a_and_com0an_1_in_wgm_mode_1_issue_78() {
        let asm = asm_program(r#"
        LDI r16, 0xfe   ; OCR0A = 0xfe;   // <- TOP value
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to PWM, Phase Correct (mode 1)
        LDI r16, 0x41   ; TCCR0A = (1 << COM0A0) | (1 << WGM00);
        OUT 0x24, r16  
        LDI r16, 0x01   ; TCCR0B = (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0xfd   ; TCNT0 = 0xfd;
        OUT 0x26, r16  "#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        // Assert that the pin callback wasn't called (thus it's disconnected)
        assert!(overrides_of(&cpu, PORT_D).is_empty());
    }

    /// upstream: `should not miss Compare Match when executing multi-cycle instruction (issue #79)`
    #[test]
    fn should_not_miss_compare_match_when_executing_multi_cycle_instruction_issue_79() {
        let asm = asm_program(r#"
        LDI r16, 0x10   ; OCR0A = 0x10;   // <- TOP value
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to normal, enable OC0A (Set on match)
        LDI r16, 0xc0   ; TCCR0A = (1 << COM0A1) | (1 << COM0A0);
        OUT 0x24, r16  
        LDI r16, 0x1    ; TCCR0B = (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0xf    ; TCNT0 = 0xf;
        OUT 0x26, r16  
        RJMP 1          ; TCNT0 will be 0x11 (RJMP takes 2 cycles)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        // Listen to Port D's internal callback
        cpu.attach_port(PORT_D);
        cpu.p.log_overrides = true;
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0x11) as i64);
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Enable)));
        // Verify that Compare Match has occured and set the OC0A pin (PD6 on ATmega328p)
        assert!(overrides_of(&cpu, PORT_D).contains(&((6) as u8, PinOverride::Set)));
    }

    /// upstream: `should only update OCR0A when TCNT0=TOP in PWM Phase Correct mode (issue #76)`
    #[test]
    fn should_only_update_ocr0a_when_tcnt0_top_in_pwm_phase_correct_mode_issue_76() {
        let asm = asm_program(r#"
        LDI r16, 0x4   ; OCR0A = 0x4;
        OUT 0x27, r16  
        ; Set waveform generation mode (WGM) to PWM, Phase Correct
        LDI r16, 0x01   ; TCCR0A = (1 << WGM00);
        OUT 0x24, r16  
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16  
        LDI r16, 0x0   ; TCNT0 = 0x0;
        OUT 0x26, r16  

        LDI r16, 0x2    ; OCR0A = 0x2; // TCNT0 should read 0x0
        OUT 0x27, r16   ; // TCNT0 should read 0x1
        NOP             ; // TCNT0 should read 0x2
        NOP             ; // TCNT0 should read 0x3
        IN r17, 0x26    ; R17 = TCNT;  // TCNT0 should read 0x4 (that's old OCR0A / TOP)
        NOP             ; // TCNT0 should read 0x3
        NOP             ; // TCNT0 should read 0x2
        NOP             ; // TCNT0 should read 0x1
        NOP             ; // TCNT0 should read 0x0
        NOP             ; // TCNT0 should read 0x1
        NOP             ; // TCNT0 should read 0x2
        IN r18, 0x26    ; R18 = TCNT; // TCNT0 should read 0x1"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (0x4) as i64);
        assert_eq!(((cpu.read_data((R18) as i32) as i64)) as i64, (0x1) as i64);
    }

    /// upstream: `should update OCR0A when TCNT0=TOP and TOP=0 in PWM Phase Correct mode (issue #119)`
    #[test]
    fn should_update_ocr0a_when_tcnt0_top_and_top_0_in_pwm_phase_correct_mode_issue_119() {
        let asm = asm_program(r#"
        ; Set waveform generation mode (WGM) to PWM, Phase Correct
        LDI r16, 0x01   ; TCCR0A = (1 << WGM00);
        OUT 0x24, r16
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16
        LDI r16, 0x0   ; TCNT0 = 0x0;
        OUT 0x26, r16

        IN r17, 0x26    ; R17 = TCNT; // TCNT0 should read 0x0
        IN r18, 0x26    ; R18 = TCNT; // TCNT0 should read 0x0
        LDI r16, 0x2    ; OCR0A = 0x2; // TCNT0 should read 0x1
        OUT 0x27, r16   ; // TCNT0 should read 0x1
        NOP             ; // TCNT0 should read 0x2
        IN r19, 0x26    ; R19 = TCNT; // TCNT0 should read 0x1"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.read_data((R18) as i32) as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.read_data((R19) as i32) as i64)) as i64, (0x1) as i64);
    }

    /// upstream: `should not overrun when TOP < current value in Phase Correct mode (issue #119)`
    #[test]
    fn should_not_overrun_when_top_current_value_in_phase_correct_mode_issue_119() {
        let asm = asm_program(r#"
        ; Set waveform generation mode (WGM) to PWM, Phase Correct
        LDI r16, 0x01   ; TCCR0A = (1 << WGM00);
        OUT 0x24, r16
        LDI r16, 0x09   ; TCCR0B = (1 << WGM02) | (1 << CS00);
        OUT 0x25, r16
        LDI r16, 0xff   ; TCNT0 = 0xff;
        OUT 0x26, r16

        IN r17, 0x26    ; R17 = TCNT; // TCNT0 should read 255"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(0);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (255) as i64);
        assert_eq!(((cpu.p.timers[0].tcnt)) as i64, (0) as i64);  // TCNT should wrap
    }

    /// upstream: `should increment 16-bit TCNT by 1`
    #[test]
    fn should_increment_16_bit_tcnt_by_1() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1H) as i32, (0x22) as u8);  // TCNT1 <- 0x2233
        cpu.write_data((TCNT1) as i32, (0x33) as u8);  // ...
        let timerLow = cpu.read_data((TCNT1) as i32) as i64;
        let timerHigh = cpu.read_data((TCNT1H) as i32) as i64;
        assert_eq!(((timerHigh << 8) | timerLow) as i64, (0x2233) as i64);
        cpu.write_data((TCCR1A) as i32, (0x0) as u8);  // WGM: Normal
        cpu.write_data((TCCR1B) as i32, (CS10) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        cpu.read_data((TCNT1) as i32);
        assert_eq!(((cpu.get16((TCNT1) as usize) as i64)) as i64, (0x2234) as i64);  // TCNT1 should increment
    }

    /// upstream: `should set OCF1A flag when timer equals OCRA (16 bit mode)`
    #[test]
    fn should_set_ocf1a_flag_when_timer_equals_ocra_16_bit_mode() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1H) as i32, (0x10) as u8);  // TCNT1 <- 0x10ee
        cpu.write_data((TCNT1) as i32, (0xee) as u8);  // ...
        cpu.write_data((OCR1AH) as i32, (0x10) as u8);  // OCR1 <- 0x10ef
        cpu.write_data((OCR1A) as i32, (0xef) as u8);  // ...
        cpu.write_data((TCCR1A) as i32, (0x0) as u8);  // WGM: Normal
        cpu.write_data((TCCR1B) as i32, (CS10) as u8);  // Set prescaler to 1
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.data[(TIFR1) as usize] as i64)) as i64, (OCF1A) as i64);  // TIFR1 should have OCF1A bit on
        assert_eq!(((cpu.pc as i64)) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (2) as i64);
    }

    /// upstream: `should generate an overflow interrupt if timer overflows and interrupts enabled`
    #[test]
    fn should_generate_an_overflow_interrupt_if_timer_overflows_and_interrupts_enabled_2() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCCR1A) as i32, (0x3) as u8);  // TCCR1A <- WGM10 | WGM11 (Fast PWM, 10-bit)
        cpu.write_data((TCCR1B) as i32, (0x9) as u8);  // TCCR1B <- WGM12 | CS10
        cpu.write_data((TIMSK1) as i32, (0x1) as u8);  // TIMSK1: TOIE1
        cpu.data[(SREG) as usize] = (0x80) as u8;  // SREG: I-------
        cpu.write_data((TCNT1H) as i32, (0x3) as u8);  // TCNT1 <- 0x3ff
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.write_data((TCNT1) as i32, (0xff) as u8);  // ...
        cpu.cycles += 1;
        cpu.tick();
        cpu.cycles += 1;
        cpu.tick();
        cpu.read_data((TCNT1) as i32);
        assert_eq!(((cpu.get16((TCNT1) as usize) as i64)) as i64, (2) as i64);
        assert_eq!(((cpu.data[(TIFR1) as usize] as i64) & TOV1) as i64, (0) as i64);
        assert_eq!(((cpu.pc as i64)) as i64, (0x1a) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (5) as i64);
    }

    /// upstream: `should reset the timer once it reaches ICR value in mode 12`
    #[test]
    fn should_reset_the_timer_once_it_reaches_icr_value_in_mode_12() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1H) as i32, (0x50) as u8);  // TCNT1 <- 0x500f
        cpu.write_data((TCNT1) as i32, (0x0f) as u8);  // ...
        cpu.write_data((ICR1H) as i32, (0x50) as u8);  // ICR1 <- 0x5010
        cpu.write_data((ICR1) as i32, (0x10) as u8);  // ...
        cpu.write_data((TCCR1B) as i32, (WGM13 | WGM12 | CS10) as u8);  // Set prescaler to 1, WGM: CTC
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (3) as u64;
        cpu.tick();
        cpu.read_data((TCNT1) as i32);
        assert_eq!(((cpu.get16((TCNT1) as usize) as i64)) as i64, (0) as i64);  // TCNT should be 0
        assert_eq!(((cpu.data[(TIFR1) as usize] as i64) & TOV1) as i64, (0) as i64);
        assert_eq!(((cpu.cycles as i64)) as i64, (3) as i64);
    }

    /// upstream: `should not update the high byte of TCNT if written after the low byte (issue #37)`
    #[test]
    fn should_not_update_the_high_byte_of_tcnt_if_written_after_the_low_byte_issue_37() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1) as i32, (0x22) as u8);
        cpu.write_data((TCNT1H) as i32, (0x55) as u8);
        cpu.cycles = (1) as u64;
        cpu.tick();
        let timerLow = cpu.read_data((TCNT1) as i32) as i64;
        let timerHigh = cpu.read_data((TCNT1H) as i32) as i64;
        assert_eq!(((timerHigh << 8) | timerLow) as i64, (0x22) as i64);
    }

    /// upstream: `reading from TCNT1H before TCNT1L should return old value (issue #37)`
    #[test]
    fn reading_from_tcnt1h_before_tcnt1l_should_return_old_value_issue_37() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCNT1H) as i32, (0xff) as u8);
        cpu.write_data((TCNT1) as i32, (0xff) as u8);
        cpu.write_data((TCCR1B) as i32, (WGM12 | CS10) as u8);  // Set prescaler to 1, WGM: CTC
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.cycles = (2) as u64;
        cpu.tick();
        // We read the high byte before the low byte, so the high byte should still have
        // the previous value:
        let timerHigh = cpu.read_data((TCNT1H) as i32) as i64;
        let timerLow = cpu.read_data((TCNT1) as i32) as i64;
        assert_eq!(((timerHigh << 8) | timerLow) as i64, (0xff00) as i64);
    }

    /// upstream: `should toggle OC1B on Compare Match`
    #[test]
    fn should_toggle_oc1b_on_compare_match() {
        let asm = asm_program(r#"
        ; Set waveform generation mode (WGM) to Normal, top 0xFFFF
        LDI r16, 0x10   ; TCCR1A = (1 << COM1B0);
        STS 0x80, r16  
        LDI r16, 0x1    ; TCCR1B = (1 << CS00);
        STS 0x81, r16  
        LDI r16, 0x0    ; OCR1BH = 0x0;
        STS 0x8B, r16  
        LDI r16, 0x4a   ; OCR1BL = 0x4a;
        STS 0x8A, r16  
        LDI r16, 0x0    ; TCNT1H = 0x0;
        STS 0x85, r16  
        LDI r16, 0x49   ; TCNT1L = 0x49;
        STS 0x84, r16  

        NOP   ; TCNT1 will be 0x49
        NOP   ; TCNT1 will be 0x4a"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(1);
        // Listen to Port B's internal callback
        cpu.attach_port(PORT_B);
        cpu.p.log_overrides = true;
        let nop_count = asm.nop_count;
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count - nop_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT1) as i32) as i64)) as i64, (0x49) as i64);
        assert!(overrides_of(&cpu, PORT_B).contains(&((2) as u8, PinOverride::Enable)));
        cpu.p.overrides.clear();
        run_instructions(&mut cpu, (1) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((TCNT1) as i32) as i64)) as i64, (0x4a) as i64);
        assert!(overrides_of(&cpu, PORT_B).contains(&((2) as u8, PinOverride::Toggle)));
    }

    /// upstream: `should only update OCR1A when TCNT1=BOTTOM in PWM Phase/Frequency Correct mode (issue #76)`
    #[test]
    fn should_only_update_ocr1a_when_tcnt1_bottom_in_pwm_phase_frequency_correct_mode_issue_76() {
        let asm = asm_program(r#"
        LDI r16, 0x0    ; OCR1AH = 0x0;
        STS 0x89, r16  
        LDI r16, 0x4   ; OCR1AL = 0x4;
        STS 0x88, r16  
        ; Set waveform generation mode (WGM) to PWM Phase/Frequency Correct mode (9)
        LDI r16, 0x01   ; TCCR1A = (1 << WGM10);
        STS 0x80, r16  
        LDI r16, 0x11   ; TCCR1B = (1 << WGM13) | (1 << CS00);
        STS 0x81, r16  
        LDI r16, 0x0    ; TCNT1H = 0x0;
        STS 0x85, r16  
        LDI r16, 0x0    ; TCNT1L = 0x0;
        STS 0x84, r16  

        LDI r16, 0x8   ; OCR1AL = 0x8; // TCNT1 should read 0x0
        STS 0x88, r16  ; // TCNT1 should read 0x2 (going up)
        LDS r17, 0x84  ; // TCNT1 should read 0x4 (going down)
        LDS r18, 0x84  ; // TCNT1 should read 0x2 (going down)
        NOP            ; // TCNT1 should read 0x0 (going up)
        NOP            ; // TCNT1 should read 0x1 (going up)
        NOP            ; // TCNT1 should read 0x2 (going up)
        NOP            ; // TCNT1 should read 0x3 (going up)
        NOP            ; // TCNT1 should read 0x4 (going up)
        NOP            ; // TCNT1 should read 0x5 (going up)
        LDS r19, 0x84  ; // TCNT1 should read 0x6 (going up)
        NOP            ; // TCNT1 should read 0x8 (going up)
        LDS r20, 0x84  ; // TCNT1 should read 0x7 (going up)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(1);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (0x4) as i64);
        assert_eq!(((cpu.read_data((R18) as i32) as i64)) as i64, (0x2) as i64);
        assert_eq!(((cpu.read_data((R19) as i32) as i64)) as i64, (0x6) as i64);
        assert_eq!(((cpu.read_data((R20) as i32) as i64)) as i64, (0x7) as i64);
    }

    /// upstream: `should update OCR1A when setting TCNT to 0 (issue #111)`
    #[test]
    fn should_update_ocr1a_when_setting_tcnt_to_0_issue_111() {
        let asm = asm_program(r#"
        CLR r1          ; r1 is our zero register
        LDI r16, 0x0    ; OCR1AH = 0x0;
        STS 0x89, r1    
        LDI r16, 0x8    ; OCR1AL = 0x8;
        STS 0x88, r16  
        ; Set waveform generation mode (WGM) to PWM Phase/Frequency Correct mode (9)
        LDI r16, 0x01   ; TCCR1A = (1 << WGM10);
        STS 0x80, r16  
        LDI r16, 0x11   ; TCCR1B = (1 << WGM13) | (1 << CS00);
        STS 0x81, r16  
        STS 0x85, r1    ; TCNT1H = 0x0;
        STS 0x84, r1    ; TCNT1L = 0x0;
        
        LDI r16, 0x5   ; OCR1AL = 0x5; // TCNT1 should read 0x0
        STS 0x88, r16  ; // TCNT1 should read 0x2 (going up)
        STS 0x84, r1   ; TCNT1L = 0x0;
        LDS r17, 0x84  ; // TCNT1 should read 0x1 (going up)
        LDS r18, 0x84  ; // TCNT1 should read 0x3 (going up)
        LDS r19, 0x84  ; // TCNT1 should read 0x5 (going down)
        LDS r20, 0x84  ; // TCNT1 should read 0x3 (going down)"#);
        let mut cpu = Cpu::new(asm.program.clone(), 8192);
        cpu.attach_timer(1);
        let on_break_throws = true;
        run_instructions(&mut cpu, (asm.instruction_count) as usize, on_break_throws);
        assert_eq!(((cpu.read_data((R17) as i32) as i64)) as i64, (0x1) as i64);
        assert_eq!(((cpu.read_data((R18) as i32) as i64)) as i64, (0x3) as i64);
        assert_eq!(((cpu.read_data((R19) as i32) as i64)) as i64, (0x5) as i64);
        assert_eq!(((cpu.read_data((R20) as i32) as i64)) as i64, (0x3) as i64);
    }

    /// upstream: `should mask the unused bits of OCR1A when using fixed top values`
    #[test]
    fn should_mask_the_unused_bits_of_ocr1a_when_using_fixed_top_values() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_timer(1);
        cpu.write_data((TCCR1A) as i32, (WGM10 | WGM11) as u8);  // WGM: FastPWM, top 0x3ff
        cpu.write_data((TCCR1B) as i32, (WGM12) as u8);
        cpu.write_data((OCR1AH) as i32, (0xff) as u8);
        cpu.write_data((OCR1A) as i32, (0xff) as u8);
        assert_eq!(((cpu.read_data((OCR1A) as i32) as i64)) as i64, (0xff) as i64);
        assert_eq!(((cpu.read_data((OCR1AH) as i32) as i64)) as i64, (0x03) as i64);
    }

    /// upstream: `should count on the falling edge of T0 when CS=110`
    #[test]
    fn should_count_on_the_falling_edge_of_t0_when_cs_110() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_port(PORT_D);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0B) as i32, (CS02 | CS01) as u8);  // Count on falling edge
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.set_pin(PORT_D, (T0) as u8, true);
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (0) as i64);
        cpu.set_pin(PORT_D, (T0) as u8, false);
        cpu.cycles = (3) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (1) as i64);
    }

    /// upstream: `should count on the rising edge of T0 when CS=111`
    #[test]
    fn should_count_on_the_rising_edge_of_t0_when_cs_111() {
        let mut cpu = Cpu::new(vec![0; 0x1000], 8192);
        cpu.attach_port(PORT_D);
        cpu.attach_timer(0);
        cpu.write_data((TCCR0B) as i32, (CS02 | CS01 | CS00) as u8);  // Count on rising edge
        cpu.cycles = (1) as u64;
        cpu.tick();
        cpu.set_pin(PORT_D, (T0) as u8, true);
        cpu.cycles = (2) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (1) as i64);
        cpu.set_pin(PORT_D, (T0) as u8, false);
        cpu.cycles = (3) as u64;
        cpu.tick();
        assert_eq!(((cpu.read_data((TCNT0) as i32) as i64)) as i64, (1) as i64);
    }

}
