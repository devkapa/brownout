//! avr8js 0.21.0's usart, spi, twi and eeprom suites, ported to the Rust port
//! (MIT, Copyright (c) Uri Shaked and contributors; see NOTICE). Upstream
//! files (github.com/wokwi/avr8js, tag v0.21.0, 898f76c): `src/peripherals/usart.spec.ts`,
//! `spi.spec.ts`, `twi.spec.ts` and `eeprom.spec.ts`.
//!
//! Converted by hand, statement by statement, in the upstream order. Every
//! upstream case is here. The callbacks the specs replace become the port's
//! host-side records: `onByteTransmit` is `Usart::tx`, `onLineTransmit`
//! `Usart::lines` (with `line_buffer` switched on), `onConfigurationChange`
//! `Usart::config_changes`, `onRxComplete` `Usart::rx_completes`; an `onByte`
//! spy is `SpiReply::Host` with `Spi::sent` recording, an `onByte` that
//! completes with a fixed byte is `SpiReply::After`; a spied or replaced TWI
//! event handler is `Twi::events` (with `TwiHandler::Host` for the replaced
//! one, whose methods do nothing). `twi.spec.ts` writes one `_replace` in
//! lower case; `asm_program` accepts either case, as upstream's assembler does.

use super::cpu::Cpu;
use super::spi::SpiReply;
use super::twi::{TwiEvent, TwiHandler};
use super::upstream_periph_tests::{asm_program, run_instructions, run_to_break};

const FREQ_16MHZ: u32 = 16_000_000;
const SREG: usize = 95;

fn cpu() -> Cpu {
    Cpu::new(vec![0; 1024], 8192)
}

/// usart.spec.ts
mod usart_spec {
    use super::*;

    const FREQ_11_0529MHZ: u32 = 11_059_200;
    const UCSR0A: i32 = 0xc0;
    const UCSR0B: i32 = 0xc1;
    const UCSR0C: i32 = 0xc2;
    const UBRR0L: i32 = 0xc4;
    const UBRR0H: i32 = 0xc5;
    const UDR0: i32 = 0xc6;
    const U2X0: u8 = 2;
    const TXEN: u8 = 8;
    const RXEN: u8 = 16;
    const UDRIE: u8 = 0x20;
    const TXCIE: u8 = 0x40;
    const RXC: u8 = 0x80;
    const TXC: u8 = 0x40;
    const UDRE: u8 = 0x20;
    const USBS: u8 = 0x08;
    const UPM0: u8 = 0x10;
    const UPM1: u8 = 0x20;
    const PC_INT_UDRE: i32 = 0x26;
    const PC_INT_TXC: i32 = 0x28;
    const UCSZ0: u8 = 2;
    const UCSZ1: u8 = 4;
    const UCSZ2: u8 = 4;

    fn usart(freq: u32) -> Cpu {
        let mut c = cpu();
        c.attach_usart(freq);
        c
    }

    fn config_changes(c: &Cpu) -> u32 {
        c.p.usart.as_ref().unwrap().config_changes
    }

    #[test]
    fn should_correctly_calculate_the_baud_rate_from_ubrr() {
        let mut cpu = usart(FREQ_11_0529MHZ);
        cpu.write_data(UBRR0H, 0);
        cpu.write_data(UBRR0L, 5);
        assert_eq!(cpu.usart_baud_rate(), 115200);
    }

    #[test]
    fn should_correctly_calculate_the_baud_rate_from_ubrr_in_double_speed_mode() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UBRR0H, 3);
        cpu.write_data(UBRR0L, 64);
        cpu.write_data(UCSR0A, U2X0);
        assert_eq!(cpu.usart_baud_rate(), 2400);
    }

    #[test]
    fn should_call_on_configuration_change_when_the_baud_rate_changes() {
        let mut cpu = usart(FREQ_16MHZ);
        let before = config_changes(&cpu);
        cpu.write_data(UBRR0H, 0);
        assert!(config_changes(&cpu) > before);
        let before = config_changes(&cpu);
        cpu.write_data(UBRR0L, 5);
        assert!(config_changes(&cpu) > before);
        let before = config_changes(&cpu);
        cpu.write_data(UCSR0A, U2X0);
        assert!(config_changes(&cpu) > before);
    }

    #[test]
    fn bits_per_char_5_when_ucsz_0() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, 0);
        assert_eq!(cpu.usart_bits_per_char(), 5);
    }

    #[test]
    fn bits_per_char_6_when_ucsz_1() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UCSZ0);
        assert_eq!(cpu.usart_bits_per_char(), 6);
    }

    #[test]
    fn bits_per_char_7_when_ucsz_2() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UCSZ1);
        assert_eq!(cpu.usart_bits_per_char(), 7);
    }

    #[test]
    fn bits_per_char_8_when_ucsz_3() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UCSZ0 | UCSZ1);
        assert_eq!(cpu.usart_bits_per_char(), 8);
    }

    #[test]
    fn bits_per_char_9_when_ucsz_7() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UCSZ0 | UCSZ1);
        cpu.write_data(UCSR0B, UCSZ2);
        assert_eq!(cpu.usart_bits_per_char(), 9);
    }

    #[test]
    fn should_call_on_configuration_change_when_bits_per_char_change() {
        let mut cpu = usart(FREQ_16MHZ);
        let before = config_changes(&cpu);
        cpu.write_data(UCSR0C, UCSZ0 | UCSZ1);
        assert!(config_changes(&cpu) > before);
        let before = config_changes(&cpu);
        cpu.write_data(UCSR0B, UCSZ2);
        assert!(config_changes(&cpu) > before);
        let before = config_changes(&cpu);
        cpu.write_data(UCSR0B, UCSZ2);
        assert_eq!(config_changes(&cpu), before);
    }

    #[test]
    fn stop_bits_1_when_usbs_0() {
        let cpu = usart(FREQ_16MHZ);
        assert_eq!(cpu.usart_stop_bits(), 1);
    }

    #[test]
    fn stop_bits_2_when_usbs_1() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, USBS);
        assert_eq!(cpu.usart_stop_bits(), 2);
    }

    #[test]
    fn parity_enabled_false_when_upm1_0() {
        let cpu = usart(FREQ_16MHZ);
        assert!(!cpu.usart_parity_enabled());
    }

    #[test]
    fn parity_enabled_true_when_upm1_1() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UPM1);
        assert!(cpu.usart_parity_enabled());
    }

    #[test]
    fn parity_odd_false_when_upm0_0() {
        let cpu = usart(FREQ_16MHZ);
        assert!(!cpu.usart_parity_odd());
    }

    #[test]
    fn parity_odd_true_when_upm0_1() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0C, UPM0);
        assert!(cpu.usart_parity_odd());
    }

    #[test]
    fn should_invoke_on_byte_transmit_when_udr0_is_written_to() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, TXEN);
        cpu.write_data(UDR0, 0x61);
        assert_eq!(cpu.p.usart.as_ref().unwrap().tx, vec![0x61]);
    }

    #[test]
    fn tx_enable_should_equal_true_when_the_transmitter_is_enabled() {
        let mut cpu = usart(FREQ_16MHZ);
        assert!(!cpu.usart_tx_enable());
        cpu.write_data(UCSR0B, TXEN);
        assert!(cpu.usart_tx_enable());
    }

    #[test]
    fn rx_enable_should_equal_true_when_the_receiver_is_enabled() {
        let mut cpu = usart(FREQ_16MHZ);
        assert!(!cpu.usart_rx_enable());
        cpu.write_data(UCSR0B, RXEN);
        assert!(cpu.usart_rx_enable());
    }

    #[test]
    fn tick_should_trigger_data_register_empty_interrupt_if_udre_is_set() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, UDRIE | TXEN);
        cpu.data[SREG] = 0x80;
        cpu.tick();
        assert_eq!(cpu.pc, PC_INT_UDRE);
        assert_eq!(cpu.cycles, 2);
        assert_eq!(cpu.data[UCSR0A as usize] & UDRE, 0);
    }

    #[test]
    fn tick_should_trigger_data_tx_complete_interrupt_if_txcie_is_set() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, TXCIE | TXEN);
        cpu.write_data(UDR0, 0x61);
        cpu.data[SREG] = 0x80;
        cpu.cycles = 1_000_000;
        cpu.tick();
        assert_eq!(cpu.pc, PC_INT_TXC);
        assert_eq!(cpu.cycles, 1_000_000 + 2);
        assert_eq!(cpu.data[UCSR0A as usize] & TXC, 0);
    }

    #[test]
    fn tick_should_not_trigger_data_tx_complete_interrupt_if_udr_was_not_written_to() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, TXCIE | TXEN);
        cpu.data[SREG] = 0x80;
        cpu.tick();
        assert_eq!(cpu.pc, 0);
        assert_eq!(cpu.cycles, 0);
    }

    #[test]
    fn tick_should_not_trigger_any_interrupt_if_interrupts_are_disabled() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, UDRIE | TXEN);
        cpu.write_data(UDR0, 0x61);
        cpu.data[SREG] = 0;
        cpu.cycles = 1_000_000;
        cpu.tick();
        assert_eq!(cpu.pc, 0);
        assert_eq!(cpu.cycles, 1_000_000);
        assert_eq!(cpu.data[UCSR0A as usize], TXC | UDRE);
    }

    fn with_lines() -> Cpu {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.p.usart.as_mut().unwrap().line_buffer = Some(String::new());
        cpu
    }

    #[test]
    fn should_call_on_line_transmit_with_the_current_line_buffer_after_every_newline() {
        let mut cpu = with_lines();
        cpu.write_data(UCSR0B, TXEN);
        for b in [0x48, 0x65, 0x6c, 0x6c, 0x6f, 0xa] {
            cpu.write_data(UDR0, b);
        }
        assert!(cpu.p.usart.as_ref().unwrap().lines.contains(&"Hello".to_string()));
    }

    #[test]
    fn should_not_call_on_line_transmit_if_no_newline_was_received() {
        let mut cpu = with_lines();
        cpu.write_data(UCSR0B, TXEN);
        cpu.write_data(UDR0, 0x48);
        cpu.write_data(UDR0, 0x69);
        assert!(cpu.p.usart.as_ref().unwrap().lines.is_empty());
    }

    #[test]
    fn should_clear_the_line_buffer_after_each_call_to_on_line_transmit() {
        let mut cpu = with_lines();
        cpu.write_data(UCSR0B, TXEN);
        for b in [0x48, 0x69, 0xa, 0x74, 0x68, 0x65, 0x72, 0x65, 0xa] {
            cpu.write_data(UDR0, b);
        }
        assert!(cpu.p.usart.as_ref().unwrap().lines.contains(&"there".to_string()));
    }

    #[test]
    fn write_byte_should_return_false_if_called_when_rx_is_busy() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, RXEN);
        cpu.write_data(UBRR0L, 103);
        assert!(cpu.usart_write_byte(10, false));
        assert!(!cpu.usart_write_byte(10, false));
        cpu.tick();
        assert!(!cpu.usart_write_byte(10, false));
    }

    #[test]
    fn should_set_the_txc_bit_after_1_04ms_when_baud_rate_set_to_9600() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, TXEN);
        cpu.write_data(UBRR0L, 103);
        cpu.write_data(UDR0, 0x48);
        cpu.cycles += 16000;
        cpu.tick();
        assert_eq!(cpu.data[UCSR0A as usize] & TXC, 0);
        cpu.cycles += 800;
        cpu.tick();
        assert_eq!(cpu.data[UCSR0A as usize] & TXC, TXC);
    }

    #[test]
    fn should_be_ready_to_receive_the_next_byte_after_1_04ms_when_baudrate_set_to_9600() {
        let mut cpu = usart(FREQ_16MHZ);
        cpu.write_data(UCSR0B, RXEN);
        cpu.write_data(UBRR0L, 103);
        assert!(cpu.usart_write_byte(0x42, false));
        cpu.cycles += 16000;
        cpu.tick();
        assert_eq!(cpu.data[UCSR0A as usize] & RXC, 0);
        assert!(cpu.p.usart.as_ref().unwrap().rx_busy);
        assert_eq!(cpu.p.usart.as_ref().unwrap().rx_completes, 0);
        cpu.cycles += 800;
        cpu.tick();
        assert_eq!(cpu.data[UCSR0A as usize] & RXC, RXC);
        assert!(!cpu.p.usart.as_ref().unwrap().rx_busy);
        assert!(cpu.p.usart.as_ref().unwrap().rx_completes > 0);
        assert_eq!(cpu.read_data(UDR0), 0x42);
        assert_eq!(cpu.read_data(UDR0), 0);
    }
}

/// spi.spec.ts
mod spi_spec {
    use super::*;

    const R17: usize = 17;
    const SPCR: i32 = 0x4c;
    const SPSR: i32 = 0x4d;
    const SPDR: i32 = 0x4e;
    const SPR0: u8 = 1;
    const SPR1: u8 = 2;
    const CPOL: u8 = 4;
    const CPHA: u8 = 8;
    const MSTR: u8 = 0x10;
    const DORD: u8 = 0x20;
    const SPE: u8 = 0x40;
    const SPIE: u8 = 0x80;
    const WCOL: u8 = 0x40;
    const SPIF: u8 = 0x80;
    const SPI2X: u8 = 1;

    fn spi() -> Cpu {
        let mut c = cpu();
        c.attach_spi(FREQ_16MHZ);
        c
    }

    fn spy(cpu: &mut Cpu) {
        let s = cpu.p.spi.as_mut().unwrap();
        s.reply = SpiReply::Host;
        s.sent = Some(Vec::new());
    }

    #[test]
    fn should_correctly_calculate_the_frequency_based_on_spcr_spsr_values() {
        let mut cpu = spi();
        let f = FREQ_16MHZ as f64;
        cpu.write_data(SPSR, 0);
        cpu.write_data(SPCR, 0);
        assert_eq!(cpu.spi_frequency(), f / 4.0);
        cpu.write_data(SPCR, SPR0);
        assert_eq!(cpu.spi_frequency(), f / 16.0);
        cpu.write_data(SPCR, SPR1);
        assert_eq!(cpu.spi_frequency(), f / 64.0);
        cpu.write_data(SPCR, SPR1 | SPR0);
        assert_eq!(cpu.spi_frequency(), f / 128.0);
        cpu.write_data(SPSR, SPI2X);
        cpu.write_data(SPCR, 0);
        assert_eq!(cpu.spi_frequency(), f / 2.0);
        cpu.write_data(SPCR, SPR0);
        assert_eq!(cpu.spi_frequency(), f / 8.0);
        cpu.write_data(SPCR, SPR1);
        assert_eq!(cpu.spi_frequency(), f / 32.0);
        cpu.write_data(SPCR, SPR1 | SPR0);
        assert_eq!(cpu.spi_frequency(), f / 64.0);
    }

    #[test]
    fn should_correctly_report_the_data_order() {
        let mut cpu = spi();
        cpu.write_data(SPCR, 0);
        assert!(!cpu.spi_lsb_first());
        cpu.write_data(SPCR, DORD);
        assert!(cpu.spi_lsb_first());
    }

    #[test]
    fn should_correctly_report_the_spi_mode() {
        // Reference quirk, ported as is: `spiMode` is 2*CPHA + CPOL where the
        // datasheet's mode is 2*CPOL + CPHA, and the spec's CPOL and CPHA
        // constants are each the other's SPCR bit (CPOL is 0x08, CPHA 0x04),
        // so the two swaps cancel here.
        let mut cpu = spi();
        cpu.write_data(SPCR, 0);
        assert_eq!(cpu.spi_mode(), 0);
        cpu.write_data(SPCR, CPHA);
        assert_eq!(cpu.spi_mode(), 1);
        cpu.write_data(SPCR, CPOL);
        assert_eq!(cpu.spi_mode(), 2);
        cpu.write_data(SPCR, CPOL | CPHA);
        assert_eq!(cpu.spi_mode(), 3);
    }

    #[test]
    fn should_indicate_slave_master_operation() {
        let mut cpu = spi();
        assert!(!cpu.spi_is_master());
        cpu.write_data(SPCR, MSTR);
        assert!(cpu.spi_is_master());
    }

    #[test]
    fn should_call_on_byte_when_initiating_an_spi_transfer_by_writing_to_spdr() {
        let mut cpu = spi();
        spy(&mut cpu);
        cpu.write_data(SPCR, SPE | MSTR);
        cpu.write_data(SPDR, 0x8f);
        assert_eq!(cpu.p.spi.as_ref().unwrap().sent, Some(vec![0x8f]));
    }

    #[test]
    fn should_ignore_spdr_writes_when_the_spe_bit_in_spcr_is_clear() {
        let mut cpu = spi();
        spy(&mut cpu);
        cpu.write_data(SPCR, MSTR);
        cpu.write_data(SPDR, 0x8f);
        assert_eq!(cpu.p.spi.as_ref().unwrap().sent, Some(vec![]));
    }

    #[test]
    fn should_transmit_a_byte_successfully_integration() {
        let asm = asm_program(&format!(
            "
      _REPLACE SPCR, {}
      _REPLACE SPDR, {}
      _REPLACE SPSR, {}
      _REPLACE DDR_SPI, 0x4 ; PORTB
      SPI_MasterInit:
        LDI r17, 0x28
        OUT DDR_SPI, r17
        LDI r17, 0x51   ; (1<<SPE)|(1<<MSTR)|(1<<SPR0)
        OUT SPCR, r17
      SPI_MasterTransmit:
        LDI r16, 0xb8 ; byte to transmit
        OUT SPDR, r16
      Wait_Transmit:
        IN r16, SPSR
        SBRS r16, 7
        RJMP Wait_Transmit
        IN r17, SPDR
        BREAK
    ",
            SPCR - 0x20,
            SPDR - 0x20,
            SPSR - 0x20
        ));
        let mut cpu = Cpu::new(asm.program, 8192);
        cpu.attach_spi(16_000_000);
        {
            let s = cpu.p.spi.as_mut().unwrap();
            s.reply = SpiReply::After(0x5b);
            s.sent = Some(Vec::new());
        }
        run_to_break(&mut cpu);
        assert!(cpu.cycles >= 128);
        assert_eq!(cpu.p.spi.as_ref().unwrap().sent, Some(vec![0xb8]));
        assert_eq!(cpu.data[R17], 0x5b);
    }

    #[test]
    fn should_set_the_wcol_bit_if_writing_to_spdr_while_spi_is_already_transmitting() {
        let mut cpu = spi();
        cpu.write_data(SPCR, SPE | MSTR);
        cpu.write_data(SPDR, 0x50);
        cpu.tick();
        assert_eq!(cpu.read_data(SPSR) & WCOL, 0);
        cpu.write_data(SPDR, 0x51);
        assert_eq!(cpu.read_data(SPSR) & WCOL, WCOL);
    }

    #[test]
    fn should_clear_the_spif_bit_and_fire_an_interrupt_when_spi_transfer_completes() {
        let mut cpu = spi();
        cpu.write_data(SPCR, SPE | SPIE | MSTR);
        cpu.write_data(SPDR, 0x50);
        cpu.data[SREG] = 0x80;
        cpu.cycles += 10;
        cpu.tick();
        assert_eq!(cpu.pc, 0);
        cpu.cycles += 100;
        cpu.tick();
        assert_eq!(cpu.data[SPSR as usize] & SPIF, 0);
        assert_eq!(cpu.pc, 0x22);
    }

    #[test]
    fn should_fire_a_pending_spi_interrupt_when_spie_flag_is_set() {
        let mut cpu = spi();
        cpu.write_data(SPCR, SPE | MSTR);
        cpu.write_data(SPDR, 0x50);
        cpu.data[SREG] = 0x80;
        cpu.cycles += 64;
        cpu.tick();
        assert_eq!(cpu.data[SPSR as usize] & SPIF, SPIF);
        assert_eq!(cpu.pc, 0);
        cpu.write_data(SPCR, SPE | MSTR | SPIE);
        cpu.tick();
        assert_eq!(cpu.pc, 0x22);
        assert_eq!(cpu.data[SPSR as usize] & SPIF, 0);
    }

    #[test]
    fn should_only_update_spdr_when_transfer_finishes_double_buffering() {
        let mut cpu = spi();
        cpu.p.spi.as_mut().unwrap().reply = SpiReply::After(0x88);
        cpu.write_data(SPCR, SPE | MSTR);
        cpu.write_data(SPDR, 0x8f);
        cpu.cycles = 10;
        cpu.tick();
        assert_eq!(cpu.read_data(SPDR), 0);
        cpu.cycles = 32;
        cpu.tick();
        assert_eq!(cpu.read_data(SPDR), 0x88);
    }
}

/// twi.spec.ts
mod twi_spec {
    use super::*;

    const R17: usize = 17;
    const TWBR: i32 = 0xb8;
    const TWSR: i32 = 0xb9;
    const TWDR: i32 = 0xbb;
    const TWCR: i32 = 0xbc;
    const TWIE: u8 = 1;
    const TWEN: u8 = 4;
    const TWSTO: u8 = 0x10;
    const TWSTA: u8 = 0x20;
    const TWEA: u8 = 0x40;
    const TWINT: u8 = 0x80;

    fn twi() -> Cpu {
        let mut c = cpu();
        c.attach_twi(FREQ_16MHZ);
        c
    }

    fn spy(cpu: &mut Cpu) {
        cpu.p.twi.as_mut().unwrap().events = Some(Vec::new());
    }

    fn called(cpu: &Cpu, e: TwiEvent) -> bool {
        cpu.p.twi.as_ref().unwrap().events.as_ref().unwrap().contains(&e)
    }

    #[test]
    fn should_correctly_calculate_the_scl_frequency_from_twbr() {
        let mut cpu = twi();
        cpu.write_data(TWBR, 0x48);
        cpu.write_data(TWSR, 0);
        assert_eq!(cpu.twi_scl_frequency(), 100000.0);
    }

    #[test]
    fn should_take_the_prescaler_into_consideration_when_calculating_scl_frequency() {
        let mut cpu = twi();
        cpu.write_data(TWBR, 0x03);
        cpu.write_data(TWSR, 0x01);
        assert_eq!(cpu.twi_scl_frequency(), 400000.0);
    }

    #[test]
    fn should_trigger_an_interrupt_if_twint_is_set() {
        let mut cpu = twi();
        cpu.write_data(TWCR, TWIE);
        cpu.data[SREG] = 0x80;
        cpu.twi_complete_start();
        cpu.tick();
        assert_eq!(cpu.pc, 0x30);
        assert_eq!(cpu.cycles, 2);
        assert_eq!(cpu.data[TWCR as usize] & TWINT, 0);
    }

    #[test]
    fn master_should_call_the_start_event_handler_when_twsta_bit_is_written_1() {
        let mut cpu = twi();
        spy(&mut cpu);
        cpu.write_data(TWCR, TWINT | TWSTA | TWEN);
        cpu.cycles += 1;
        cpu.tick();
        assert!(called(&cpu, TwiEvent::Start { repeated: false }));
    }

    #[test]
    fn master_should_connect_successfully_in_case_of_repeated_start_issue_91() {
        let mut cpu = twi();
        cpu.write_data(TWCR, TWINT | TWSTA | TWEN);
        cpu.cycles += 1;
        cpu.tick();
        spy(&mut cpu);
        cpu.write_data(TWCR, TWINT | TWSTA | TWEN);
        cpu.cycles += 1;
        cpu.tick();
        assert!(called(&cpu, TwiEvent::Start { repeated: true }));
        cpu.write_data(TWDR, 0x80);
        cpu.write_data(TWCR, TWINT | TWEN);
        cpu.cycles += 1;
        cpu.tick();
        assert!(called(&cpu, TwiEvent::Connect { addr: 0x40, write: true }));
    }

    fn host_twi(src: &str) -> Cpu {
        let asm = asm_program(src);
        let mut cpu = Cpu::new(asm.program, 8192);
        cpu.attach_twi(FREQ_16MHZ);
        let t = cpu.p.twi.as_mut().unwrap();
        t.handler = TwiHandler::Host;
        t.events = Some(Vec::new());
        cpu
    }

    const WAIT_AND_ERROR: &str = "
        loop:
        jmp loop
        wait_for_twint:
        lds r16, TWCR
        andi r16, TWINT
        breq wait_for_twint
        ret
        error:
        break
    ";

    #[test]
    fn master_should_successfully_transmit_a_byte_to_a_slave() {
        let src = format!(
            "
        _REPLACE TWSR, {TWSR}
        _REPLACE TWDR, {TWDR}
        _REPLACE TWCR, {TWCR}
        _REPLACE TWEN, {TWEN}
        _REPLACE TWSTO, {TWSTO}
        _REPLACE TWSTA, {TWSTA}
        _REPLACE TWINT, {TWINT}
        _REPLACE START, 0x8
        _REPLACE MT_SLA_ACK, 0x18
        _REPLACE MT_DATA_ACK, 0x28
        ldi r16, TWEN
        sbr r16, TWSTA
        sbr r16, TWINT
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, START
        brne error
        _REPLACE SLA_W, 0x44
        ldi r16, SLA_W
        sts TWDR, r16
        ldi r16, TWINT
        sbr r16, TWEN
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, MT_SLA_ACK
        brne error
        _replace DATA, 0x55
        ldi r16, DATA
        sts TWDR, r16
        ldi r16, TWINT
        sbr r16, TWEN
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, MT_DATA_ACK
        brne error
        ldi r16, TWINT
        sbr r16, TWEN
        sbr r16, TWSTO
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, 0xf8
        brne error
        ldi r17, 0x42
        {WAIT_AND_ERROR}"
        );
        let mut cpu = host_twi(&src);
        run_instructions(&mut cpu, 4, false);
        assert!(called(&cpu, TwiEvent::Start { repeated: false }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_start();
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::Connect { addr: 0x22, write: true }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_connect(true);
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::WriteByte(0x55)));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_write(true);
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::Stop));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_stop();
        run_instructions(&mut cpu, 16, false);
        assert_eq!(cpu.data[R17], 0x42);
    }

    #[test]
    fn master_should_successfully_receive_a_byte_from_a_slave() {
        let src = format!(
            "
        _REPLACE TWSR, {TWSR}
        _REPLACE TWDR, {TWDR}
        _REPLACE TWCR, {TWCR}
        _REPLACE TWEN, {TWEN}
        _REPLACE TWSTO, {TWSTO}
        _REPLACE TWSTA, {TWSTA}
        _REPLACE TWEA, {TWEA}
        _REPLACE TWINT, {TWINT}
        _REPLACE START, 0x8
        _REPLACE MT_SLAR_ACK, 0x40
        _REPLACE MT_DATA_RECV, 0x50
        _REPLACE MT_DATA_RECV_NACK, 0x58
        ldi r16, TWEN
        sbr r16, TWSTA
        sbr r16, TWINT
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        ldi r18, START
        cpse r16, r18
        jmp error
        _REPLACE SLA_R, 0xa1
        ldi r16, SLA_R
        sts TWDR, r16
        ldi r16, TWINT
        sbr r16, TWEN
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, MT_SLAR_ACK
        brne error
        ldi r16, TWINT
        sbr r16, TWEA
        sbr r16, TWEN
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, MT_DATA_RECV
        brne error
        lds r16, TWDR
        cpi r16, 0x66
        brne error
        ldi r16, TWINT
        sbr r16, TWEN
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, MT_DATA_RECV_NACK
        brne error
        lds r16, TWDR
        cpi r16, 0x77
        brne error
        ldi r16, TWINT
        sbr r16, TWEN
        sbr r16, TWSTO
        sts TWCR, r16
        call wait_for_twint
        lds r16, TWSR
        andi r16, 0xf8
        cpi r16, 0xf8
        brne error
        ldi r17, 0x42
        {WAIT_AND_ERROR}"
        );
        let mut cpu = host_twi(&src);
        run_instructions(&mut cpu, 4, false);
        assert!(called(&cpu, TwiEvent::Start { repeated: false }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_start();
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::Connect { addr: 0x50, write: false }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_connect(true);
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::ReadByte { ack: true }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_read(0x66);
        run_instructions(&mut cpu, 16, false);
        assert!(called(&cpu, TwiEvent::ReadByte { ack: false }));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_read(0x77);
        run_instructions(&mut cpu, 24, false);
        assert!(called(&cpu, TwiEvent::Stop));
        run_instructions(&mut cpu, 16, false);
        cpu.twi_complete_stop();
        run_instructions(&mut cpu, 16, false);
        assert_eq!(cpu.data[R17], 0x42);
    }
}

/// eeprom.spec.ts
mod eeprom_spec {
    use super::*;

    const EECR: i32 = 0x3f;
    const EEDR: i32 = 0x40;
    const EEARL: i32 = 0x41;
    const EEARH: i32 = 0x42;
    const EERE: u8 = 1;
    const EEPE: u8 = 2;
    const EEMPE: u8 = 4;
    const EERIE: u8 = 8;

    fn eeprom_cpu(prog: Vec<u16>) -> Cpu {
        let mut c = Cpu::new(prog, 8192);
        c.attach_eeprom(vec![0xff; 1024]);
        c
    }

    fn mem(cpu: &Cpu) -> &[u8] {
        cpu.eeprom_memory()
    }

    #[test]
    fn should_return_0xff_when_reading_from_an_empty_location() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        cpu.write_data(EEARL, 0);
        cpu.write_data(EEARH, 0);
        cpu.write_data(EECR, EERE);
        cpu.tick();
        assert_eq!(cpu.cycles, 4);
        assert_eq!(cpu.data[EEDR as usize], 0xff);
    }

    #[test]
    fn should_return_the_value_stored_at_the_given_eeprom_address() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        cpu.eeprom_memory_mut()[0x250] = 0x42;
        cpu.write_data(EEARL, 0x50);
        cpu.write_data(EEARH, 0x2);
        cpu.write_data(EECR, EERE);
        cpu.tick();
        assert_eq!(cpu.data[EEDR as usize], 0x42);
    }

    fn write_15(cpu: &mut Cpu) {
        cpu.write_data(EEDR, 0x55);
        cpu.write_data(EEARL, 15);
        cpu.write_data(EEARH, 0);
        cpu.write_data(EECR, EEMPE);
    }

    #[test]
    fn should_write_a_byte_to_the_given_eeprom_address() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 2);
        assert_eq!(mem(&cpu)[15], 0x55);
        assert_eq!(cpu.data[EECR as usize] & EEPE, EEPE);
    }

    fn sbi_program(eepm_bit: u8) -> super::super::upstream_periph_tests::Asm {
        asm_program(&format!(
            "
        _REPLACE EEARL, {}
        _REPLACE EEDR, {}
        _REPLACE EECR, {}
        LDI r16, 0x55
        OUT EEDR, r16
        LDI r16, 9
        OUT EEARL, r16
        SBI EECR, {eepm_bit}
        SBI EECR, 2     ; EECR |= EEMPE
        SBI EECR, 1     ; EECR |= EEPE
      ",
            EEARL - 0x20,
            EEDR - 0x20,
            EECR - 0x20
        ))
    }

    #[test]
    fn should_not_erase_the_memory_when_writing_if_eepm1_is_high() {
        let asm = sbi_program(5);
        let mut cpu = eeprom_cpu(asm.program);
        cpu.eeprom_memory_mut()[9] = 0x0f;
        run_instructions(&mut cpu, asm.instruction_count, true);
        assert_eq!(mem(&cpu)[9], 0x05);
    }

    #[test]
    fn should_clear_the_eepe_bit_and_fire_an_interrupt_when_write_has_been_completed() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.data[SREG] = 0x80;
        cpu.write_data(EECR, EEPE | EERIE);
        cpu.cycles += 1000;
        cpu.tick();
        assert_eq!(cpu.data[EECR as usize] & EEPE, EEPE);
        assert_eq!(cpu.pc, 0);
        cpu.cycles += 10_000_000;
        cpu.tick();
        assert_eq!(mem(&cpu)[15], 0x55);
        assert_eq!(cpu.data[EECR as usize] & EEPE, 0);
        assert_eq!(cpu.pc, 0x2c);
    }

    #[test]
    fn should_fire_an_interrupt_when_there_is_a_pending_interrupt_and_the_interrupt_flag_is_enabled_issue_110() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.data[SREG] = 0x80;
        cpu.write_data(EECR, EEPE);
        cpu.cycles += 1000;
        cpu.tick();
        assert_eq!(cpu.data[EECR as usize] & EEPE, EEPE);
        assert_eq!(cpu.pc, 0);
        cpu.cycles += 10_000_000;
        cpu.tick();
        assert_eq!(mem(&cpu)[15], 0x55);
        assert_eq!(cpu.data[EECR as usize] & EEPE, 0);
        cpu.write_data(EECR, EERIE);
        cpu.tick();
        assert_eq!(cpu.pc, 0x2c);
    }

    #[test]
    fn should_skip_the_write_if_eempe_is_clear() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.cycles = 8;
        cpu.tick();
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 8);
        assert_eq!(mem(&cpu)[15], 0xff);
        assert_eq!(cpu.data[EECR as usize] & EEPE, 0);
    }

    #[test]
    fn should_skip_the_write_if_another_write_is_already_in_progress() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 2);
        cpu.write_data(EEDR, 0x66);
        cpu.write_data(EEARL, 16);
        cpu.write_data(EEARH, 0);
        cpu.write_data(EECR, EEMPE);
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 2);
        assert_eq!(mem(&cpu)[15], 0x55);
        assert_eq!(mem(&cpu)[16], 0xff);
    }

    #[test]
    fn should_write_two_bytes_successfully() {
        let mut cpu = eeprom_cpu(vec![0; 0x1000]);
        write_15(&mut cpu);
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 2);
        cpu.cycles += 10_000_000;
        cpu.tick();
        cpu.write_data(EEDR, 0x66);
        cpu.write_data(EEARL, 16);
        cpu.write_data(EEARH, 0);
        cpu.write_data(EECR, EEMPE);
        cpu.write_data(EECR, EEPE);
        cpu.tick();
        assert_eq!(cpu.cycles, 10_000_004);
        assert_eq!(mem(&cpu)[15], 0x55);
        assert_eq!(mem(&cpu)[16], 0x66);
    }

    #[test]
    fn should_only_erase_the_memory_when_eepm0_is_high() {
        let asm = sbi_program(4);
        let mut cpu = eeprom_cpu(asm.program);
        cpu.eeprom_memory_mut()[9] = 0x22;
        run_instructions(&mut cpu, asm.instruction_count, true);
        assert_eq!(mem(&cpu)[9], 0xff);
    }
}
