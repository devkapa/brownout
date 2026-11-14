//! Unit tests for R6: the pin-facing peripherals (PWM, ADC, UARTs, SPI, I2C
//! and the pin state they drive). Expected values come from the RP2040
//! datasheet, derived in each test from field positions and the documented
//! behaviour. Where rp2040js 1.3.3 departs from the datasheet the port follows
//! it and a `reference_quirk_*` test pins both.
//!
//! Upstream suite ported by hand, tag v1.3.3 (MIT; see NOTICE):
//! `src/peripherals/uart.spec.ts` (two tests, in `mod uart`). rp2040js has no
//! other tests for these blocks.

use super::sio::{PIN_HIGH, PIN_LOW};
use super::*;

fn rd(m: &mut Rp2040, a: u32) -> u32 {
    m.read_u32(a)
}
fn wr(m: &mut Rp2040, a: u32, v: u32) {
    m.write_u32(a, v)
}
fn ns(m: &mut Rp2040, nanos: f64) {
    m.clock_tick(nanos);
}

const IO_BANK0: u32 = 0x4001_4000;
const SET: u32 = 0x2000;

fn pin_ctrl(pin: u32) -> u32 {
    IO_BANK0 + 8 * pin + 4
}

mod pwm {
    use super::*;

    const PWM: u32 = 0x4005_0000;
    const CSR: u32 = 0x00;
    const DIV: u32 = 0x04;
    const CTR: u32 = 0x08;
    const CC: u32 = 0x0c;
    const TOP: u32 = 0x10;
    const EN: u32 = 0xa0;
    const INTR: u32 = 0xa4;
    const INTE: u32 = 0xa8;
    const INTF: u32 = 0xac;
    const INTS: u32 = 0xb0;
    fn ch(n: u32, reg: u32) -> u32 {
        PWM + 0x14 * n + reg
    }

    fn events(m: &Rp2040, pin: u32) -> Vec<(u32, f64)> {
        m.pin_events.iter().filter(|e| e.pin == pin).map(|e| (e.state, e.nanos)).collect()
    }

    #[test]
    fn reset_values_match_the_datasheet() {
        // Datasheet 4.5.3: CSR 0, DIV INT=1 (0x10), CTR 0, CC 0, TOP 0xffff.
        let mut m = Rp2040::new();
        for n in 0..8 {
            assert_eq!(rd(&mut m, ch(n, CSR)), 0);
            assert_eq!(rd(&mut m, ch(n, DIV)), 1 << 4);
            assert_eq!(rd(&mut m, ch(n, CTR)), 0);
            assert_eq!(rd(&mut m, ch(n, CC)), 0);
            assert_eq!(rd(&mut m, ch(n, TOP)), 0xffff);
        }
        assert_eq!(rd(&mut m, PWM + INTR), 0);
        assert!(m.stop.is_none());
    }

    #[test]
    fn frequency_and_duty_follow_top_cc_and_div() {
        // Datasheet 4.5.2: f = clk_sys / (TOP + 1) / DIV, duty = CC_A / (TOP + 1).
        // TOP 99, CC_A 25, DIV 1 at 125 MHz: period 100 cycles = 800 ns, high 200 ns.
        let mut m = Rp2040::new();
        wr(&mut m, pin_ctrl(0), 4);
        wr(&mut m, ch(0, TOP), 99);
        wr(&mut m, ch(0, CC), 25);
        m.pin_events.clear();
        wr(&mut m, ch(0, CSR), 1);
        ns(&mut m, 1800.0);
        assert_eq!(
            events(&m, 0),
            vec![(PIN_HIGH, 800.0), (PIN_LOW, 1000.0), (PIN_HIGH, 1600.0), (PIN_LOW, 1800.0)]
        );
    }

    #[test]
    fn divider_scales_the_period() {
        // DIV INT 2, FRAC 8/16 = 2.5; TOP 9 -> 10 counts * 2.5 = 25 cycles = 200 ns.
        let mut m = Rp2040::new();
        wr(&mut m, pin_ctrl(2), 4);
        wr(&mut m, ch(1, DIV), (2 << 4) | 8);
        wr(&mut m, ch(1, TOP), 9);
        wr(&mut m, ch(1, CC), 5);
        m.pin_events.clear();
        wr(&mut m, ch(1, CSR), 1);
        ns(&mut m, 400.0);
        assert_eq!(events(&m, 2), vec![(PIN_HIGH, 200.0), (PIN_LOW, 300.0), (PIN_HIGH, 400.0)]);
    }

    #[test]
    fn output_invert_swaps_the_levels() {
        // CSR.A_INV (bit 2): the output is high except while the compare window runs.
        let mut m = Rp2040::new();
        wr(&mut m, pin_ctrl(0), 4);
        wr(&mut m, ch(0, TOP), 99);
        wr(&mut m, ch(0, CC), 25);
        m.pin_events.clear();
        wr(&mut m, ch(0, CSR), 1 | (1 << 2));
        ns(&mut m, 1000.0);
        let e = events(&m, 0);
        assert_eq!(e[0].0, PIN_HIGH, "A_INV: the pin leaves the window high");
        assert_eq!(e[1], (PIN_LOW, 800.0));
        assert_eq!(e[2], (PIN_HIGH, 1000.0));
    }

    #[test]
    fn pins_map_a_and_b_of_each_slice() {
        // Datasheet table 525: slice n drives GPIO 2n (A) and 2n+1 (B), and
        // slice n < 7 also GPIO 16+2n and 16+2n+1.
        let mut m = Rp2040::new();
        for pin in [6, 7, 22, 23] {
            wr(&mut m, pin_ctrl(pin), 4);
        }
        wr(&mut m, ch(3, TOP), 9);
        wr(&mut m, ch(3, CC), 3 | (3 << 16));
        m.pin_events.clear();
        wr(&mut m, ch(3, CSR), 1);
        ns(&mut m, 100.0);
        for pin in [6, 7, 22, 23] {
            assert_eq!(events(&m, pin).first().map(|e| e.0), Some(PIN_HIGH), "pin {pin}");
        }
    }

    #[test]
    fn wrap_raises_interrupt_and_dreq() {
        // Datasheet 4.5.3: INTR bit n is slice n's wrap; INTS = (INTR & INTE) | INTF.
        let mut m = Rp2040::new();
        wr(&mut m, ch(2, TOP), 9);
        wr(&mut m, ch(2, CSR), 1);
        ns(&mut m, 79.0);
        assert_eq!(rd(&mut m, PWM + INTR), 0, "10 cycles = 80 ns: not yet");
        ns(&mut m, 1.0);
        assert_eq!(rd(&mut m, PWM + INTR), 1 << 2);
        assert_eq!(rd(&mut m, PWM + INTS), 0, "masked");
        assert_eq!(m.cpu.pending_interrupts & (1 << 4), 0);
        wr(&mut m, PWM + INTE, 1 << 2);
        assert_eq!(rd(&mut m, PWM + INTS), 1 << 2);
        assert_ne!(m.cpu.pending_interrupts & (1 << 4), 0, "IRQ 4 = PWM_IRQ_WRAP");
        wr(&mut m, PWM + INTR, 1 << 2);
        assert_eq!(rd(&mut m, PWM + INTR), 0);
        wr(&mut m, PWM + INTF, 1 << 5);
        assert_eq!(rd(&mut m, PWM + INTS), (1 << 5) | 0);
        assert!(m.periph.dma.dreq[(super::super::dma::dreq::PWM_WRAP0 + 2) as usize]);
    }

    #[test]
    fn en_register_starts_slices_together() {
        // Datasheet: the EN register aliases every slice's CSR.EN.
        let mut m = Rp2040::new();
        wr(&mut m, PWM + EN, 0b101);
        assert_eq!(rd(&mut m, ch(0, CSR)) & 1, 1);
        assert_eq!(rd(&mut m, ch(1, CSR)) & 1, 0);
        assert_eq!(rd(&mut m, ch(2, CSR)) & 1, 1);
        wr(&mut m, PWM + EN, 0);
        assert_eq!(rd(&mut m, ch(0, CSR)) & 1, 0);
    }

    #[test]
    fn atomic_set_alias_reaches_the_slice_registers() {
        let mut m = Rp2040::new();
        wr(&mut m, ch(0, CSR) + SET, 1 << 3);
        assert_eq!(rd(&mut m, ch(0, CSR)), 1 << 3);
    }

    #[test]
    fn reference_quirk_en_register_reads_zero() {
        // Datasheet: EN reads back the slice enables. The reference's `en` is
        // a setter-only accessor, so the sum is zero.
        let mut m = Rp2040::new();
        wr(&mut m, PWM + EN, 0xff);
        assert_eq!(rd(&mut m, ch(7, CSR)) & 1, 1);
        assert_eq!(rd(&mut m, PWM + EN), 0 /* datasheet: 0xff */);
    }

    #[test]
    fn reference_quirk_ph_adv_does_not_advance_the_counter() {
        // Datasheet: writing CSR.PH_ADV (bit 7) advances a stopped counter by
        // one. The reference masks the bit off before testing it.
        let mut m = Rp2040::new();
        wr(&mut m, ch(0, CSR), 1 << 7);
        assert_eq!(rd(&mut m, ch(0, CTR)), 0 /* datasheet: 1 */);
        assert_eq!(rd(&mut m, ch(0, CSR)), 0, "the bit is self-clearing");
    }

    #[test]
    fn reference_quirk_writing_div_starts_a_stopped_slice() {
        // Datasheet: a slice with EN = 0 does not count. The reference's
        // `Timer32.prescaler` setter re-enables the timer.
        let mut m = Rp2040::new();
        wr(&mut m, ch(0, DIV), 2 << 4);
        ns(&mut m, 1000.0);
        assert_ne!(rd(&mut m, ch(0, CTR)), 0 /* datasheet: 0 */);
    }

    #[test]
    fn reference_quirk_b_pin_input_never_counts() {
        // Datasheet: in B-rising-edge mode (DIVMODE = 2) each rising edge on B
        // advances the counter by one (DIV 1). The reference's gpioOnInput
        // returns whenever any direction bit is set, which is always.
        let mut m = Rp2040::new();
        wr(&mut m, pin_ctrl(1), 4);
        wr(&mut m, 0x4001_c000 + 4 + 4, 0x40 | 0x16); // pad: input enable on GPIO1
        wr(&mut m, ch(0, CSR), 1 | (2 << 4));
        m.set_input_value(1, true);
        m.set_input_value(1, false);
        m.set_input_value(1, true);
        assert_eq!(rd(&mut m, ch(0, CTR)), 0 /* datasheet: 2 */);
        // With the direction word cleared the hook does run, as written.
        m.periph.pwm.gpio_direction = 0;
        m.set_input_value(1, false);
        m.set_input_value(1, true);
        assert_eq!(rd(&mut m, ch(0, CTR)), 1);
    }

    #[test]
    fn reference_quirk_gpio1_remembers_its_state_after_reset() {
        // The first gpioSetDir compares the double 0xffffffff with int32 -1,
        // so the reset re-evaluates GPIO1 only; every other pin's first drive
        // update reports Input -> its real state.
        let m = Rp2040::new();
        assert_eq!(m.sio.gpio[1].last_value, 4);
        assert_eq!(m.sio.gpio[0].last_value, 2);
        assert_eq!(m.periph.pwm.gpio_direction, -1);
    }
}

mod adc {
    use super::*;

    const ADC: u32 = 0x4004_c000;
    const CS: u32 = 0x00;
    const RESULT: u32 = 0x04;
    const FCS: u32 = 0x08;
    const FIFO: u32 = 0x0c;
    const DIV: u32 = 0x10;
    const INTR: u32 = 0x14;
    const INTE: u32 = 0x18;
    const INTF: u32 = 0x1c;
    const INTS: u32 = 0x20;
    const EN: u32 = 1;
    const START_ONE: u32 = 1 << 2;
    const START_MANY: u32 = 1 << 3;
    const READY: u32 = 1 << 8;
    const AINSEL: u32 = 12;
    const RROBIN: u32 = 16;
    const FCS_EN: u32 = 1;
    const FCS_SHIFT: u32 = 1 << 1;
    const FCS_DREQ_EN: u32 = 1 << 3;
    const FCS_EMPTY: u32 = 1 << 8;
    const FCS_FULL: u32 = 1 << 9;
    const FCS_UNDER: u32 = 1 << 10;
    const FCS_OVER: u32 = 1 << 11;

    fn level(m: &mut Rp2040) -> u32 {
        (rd(m, ADC + FCS) >> 16) & 0xf
    }

    #[test]
    fn a_conversion_takes_two_microseconds() {
        // Datasheet 4.9.2: 96 cycles of the 48 MHz ADC clock = 2 us per sample.
        let mut m = Rp2040::new();
        m.adc_set_channel(0, 1234.0);
        wr(&mut m, ADC + CS, EN | START_ONE);
        assert_eq!(rd(&mut m, ADC + CS) & READY, 0);
        ns(&mut m, 1999.0);
        assert_eq!(rd(&mut m, ADC + CS) & READY, 0);
        ns(&mut m, 1.0);
        assert_ne!(rd(&mut m, ADC + CS) & READY, 0);
        assert_eq!(rd(&mut m, ADC + RESULT), 1234);
    }

    #[test]
    fn ainsel_picks_the_channel() {
        let mut m = Rp2040::new();
        for c in 0..5 {
            m.adc_set_channel(c, 100.0 + c as f64);
        }
        for c in 0..5u32 {
            wr(&mut m, ADC + CS, EN | START_ONE | (c << AINSEL));
            ns(&mut m, 2000.0);
            assert_eq!(rd(&mut m, ADC + RESULT), 100 + c);
        }
    }

    #[test]
    fn fifo_collects_samples_and_shift_gives_eight_bits() {
        // FCS.SHIFT (datasheet): right-shift samples to 8 bits.
        let mut m = Rp2040::new();
        m.adc_set_channel(1, 0xabc as f64);
        wr(&mut m, ADC + FCS, FCS_EN);
        assert_ne!(rd(&mut m, ADC + FCS) & FCS_EMPTY, 0);
        wr(&mut m, ADC + CS, EN | START_ONE | (1 << AINSEL));
        ns(&mut m, 2000.0);
        assert_eq!(level(&mut m), 1);
        assert_eq!(rd(&mut m, ADC + FCS) & FCS_EMPTY, 0);
        assert_eq!(rd(&mut m, ADC + FIFO), 0xabc);
        wr(&mut m, ADC + FCS, FCS_EN | FCS_SHIFT);
        wr(&mut m, ADC + CS, EN | START_ONE | (1 << AINSEL));
        ns(&mut m, 2000.0);
        assert_eq!(rd(&mut m, ADC + FIFO), 0xab);
    }

    #[test]
    fn fifo_full_and_overflow() {
        // Four-entry FIFO; a fifth sample sets FCS.OVER (write-1-to-clear).
        let mut m = Rp2040::new();
        m.adc_set_channel(0, 7.0);
        wr(&mut m, ADC + FCS, FCS_EN);
        wr(&mut m, ADC + CS, EN | START_MANY); // DIV 0: one sample per 2 us
        ns(&mut m, 8000.0);
        assert_eq!(level(&mut m), 4);
        assert_ne!(rd(&mut m, ADC + FCS) & FCS_FULL, 0);
        assert_eq!(rd(&mut m, ADC + FCS) & FCS_OVER, 0);
        ns(&mut m, 2000.0);
        assert_ne!(rd(&mut m, ADC + FCS) & FCS_OVER, 0);
        wr(&mut m, ADC + FCS, FCS_EN | FCS_OVER);
        assert_eq!(rd(&mut m, ADC + FCS) & FCS_OVER, 0);
    }

    #[test]
    fn reading_an_empty_fifo_sets_under() {
        let mut m = Rp2040::new();
        assert_eq!(rd(&mut m, ADC + FIFO), 0);
        assert_ne!(rd(&mut m, ADC + FCS) & FCS_UNDER, 0);
        wr(&mut m, ADC + FCS, FCS_UNDER);
        assert_eq!(rd(&mut m, ADC + FCS) & FCS_UNDER, 0);
    }

    #[test]
    fn continuous_sampling_follows_the_divider() {
        // Datasheet: one sample per (1 + INT + FRAC/256) cycles of 48 MHz.
        // INT 191: 192 cycles = 4 us between starts; the first lands at 2 us.
        let mut m = Rp2040::new();
        m.adc_set_channel(0, 5.0);
        wr(&mut m, ADC + FCS, FCS_EN);
        wr(&mut m, ADC + DIV, 191 << 8);
        wr(&mut m, ADC + CS, EN | START_MANY);
        ns(&mut m, 1999.0);
        assert_eq!(level(&mut m), 0);
        ns(&mut m, 1.0);
        assert_eq!(level(&mut m), 1);
        ns(&mut m, 4000.0);
        assert_eq!(level(&mut m), 2);
        ns(&mut m, 3999.0);
        assert_eq!(level(&mut m), 2);
        ns(&mut m, 1.0);
        assert_eq!(level(&mut m), 3);
    }

    #[test]
    fn fifo_interrupt_follows_the_threshold() {
        // INTR is set while the FIFO level >= THRESH; INTS = (INTR & INTE) | INTF.
        let mut m = Rp2040::new();
        m.adc_set_channel(0, 9.0);
        wr(&mut m, ADC + FCS, FCS_EN | (1 << 24));
        wr(&mut m, ADC + INTE, 1);
        assert_eq!(rd(&mut m, ADC + INTR), 0);
        assert_eq!(m.cpu.pending_interrupts & (1 << 22), 0);
        wr(&mut m, ADC + CS, EN | START_ONE);
        ns(&mut m, 2000.0);
        assert_eq!(rd(&mut m, ADC + INTR), 1);
        assert_eq!(rd(&mut m, ADC + INTS), 1);
        assert_ne!(m.cpu.pending_interrupts & (1 << 22), 0, "IRQ 22 = ADC_IRQ_FIFO");
        wr(&mut m, ADC + INTE, 0);
        wr(&mut m, ADC + INTF, 1);
        assert_eq!(rd(&mut m, ADC + INTS), 1);
    }

    #[test]
    fn dreq_follows_the_fifo_level() {
        let mut m = Rp2040::new();
        wr(&mut m, ADC + FCS, FCS_EN | FCS_DREQ_EN | (1 << 24));
        m.adc_set_channel(0, 1.0);
        wr(&mut m, ADC + CS, EN | START_ONE);
        ns(&mut m, 2000.0);
        assert!(m.periph.dma.dreq[super::super::dma::dreq::ADC as usize]);
        rd(&mut m, ADC + FIFO);
        assert!(!m.periph.dma.dreq[super::super::dma::dreq::ADC as usize]);
    }

    #[test]
    fn reference_quirk_cs_write_clears_fcs_under() {
        // Datasheet: CS.ERR_STICKY (bit 10) is write-1-to-clear in CS and
        // FCS.UNDER is only cleared through FCS. The reference clears the FCS
        // bit with the same position when CS is written.
        let mut m = Rp2040::new();
        rd(&mut m, ADC + FIFO);
        assert_ne!(rd(&mut m, ADC + FCS) & FCS_UNDER, 0);
        wr(&mut m, ADC + CS, 1 << 10);
        assert_eq!(rd(&mut m, ADC + FCS) & FCS_UNDER, 0 /* datasheet: still set */);
    }

    #[test]
    fn reference_quirk_start_one_reads_back_set() {
        // Datasheet: START_ONE self-clears. The reference stores it.
        let mut m = Rp2040::new();
        wr(&mut m, ADC + CS, EN | START_ONE);
        assert_ne!(rd(&mut m, ADC + CS) & START_ONE, 0 /* datasheet: 0 */);
    }

    #[test]
    fn reference_quirk_round_robin_cannot_leave_channel_zero() {
        // Datasheet: RROBIN = channels 0,1,2 steps 0 -> 1 -> 2 -> 0. The
        // reference masks the next channel with 12, so 1 and 2 become 0.
        let mut m = Rp2040::new();
        wr(&mut m, ADC + CS, EN | START_ONE | (0b111 << RROBIN));
        ns(&mut m, 2000.0);
        assert_eq!((rd(&mut m, ADC + CS) >> AINSEL) & 7, 0 /* datasheet: 1 */);
        // Channel 4 survives the mask, so a round robin over {4} works.
        let mut m = Rp2040::new();
        wr(&mut m, ADC + CS, EN | START_ONE | (0b10000 << RROBIN));
        ns(&mut m, 2000.0);
        assert_eq!((rd(&mut m, ADC + CS) >> AINSEL) & 7, 4);
    }

    #[test]
    fn reference_quirk_fifo_read_does_not_refresh_the_interrupt() {
        // Datasheet: the FIFO interrupt deasserts when the level drops below
        // THRESH. The reference only recomputes the line on a push or a
        // register write, so the pending bit stays after the pull.
        let mut m = Rp2040::new();
        wr(&mut m, ADC + FCS, FCS_EN | (1 << 24));
        wr(&mut m, ADC + INTE, 1);
        wr(&mut m, ADC + CS, EN | START_ONE);
        ns(&mut m, 2000.0);
        assert_ne!(m.cpu.pending_interrupts & (1 << 22), 0);
        rd(&mut m, ADC + FIFO);
        assert_eq!(rd(&mut m, ADC + INTS), 0);
        assert_ne!(m.cpu.pending_interrupts & (1 << 22), 0 /* datasheet: clear */);
    }
}

mod uart {
    use super::super::dma::dreq;
    use super::*;

    const UART0: u32 = 0x4003_4000;
    const UART1: u32 = 0x4003_8000;
    const DR: u32 = 0x0;
    const FR: u32 = 0x18;
    const IBRD: u32 = 0x24;
    const FBRD: u32 = 0x28;
    const LCR_H: u32 = 0x2c;
    const CR: u32 = 0x30;
    const IMSC: u32 = 0x38;
    const RIS: u32 = 0x3c;
    const MIS: u32 = 0x40;
    const ICR: u32 = 0x44;
    const RXINTR: u32 = 1 << 4;
    const TXINTR: u32 = 1 << 5;

    // ---- src/peripherals/uart.spec.ts (both tests), ported by hand -------------

    #[test]
    fn upstream_should_correctly_return_word_length_based_on_uartlcr_h_value() {
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + LCR_H, 0x70);
        assert_eq!(m.periph.uart[0].word_length(), 8);
    }

    #[test]
    fn upstream_should_correctly_calculate_the_baud_rate_based_on_ibrd_fbrd() {
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + IBRD, 67); // datasheet 4.2.7.1 example values
        wr(&mut m, UART0 + FBRD, 52);
        assert_eq!(m.periph.uart[0].baud_rate(), 115207.0);
    }

    // ---- datasheet-derived ----------------------------------------------------------

    #[test]
    fn reset_values() {
        // Datasheet 4.2.8: UARTFR 0x90 (TXFE, RXFE), UARTCR 0x300 (TXE, RXE),
        // IBRD/FBRD/LCR_H/IMSC 0.
        let mut m = Rp2040::new();
        for base in [UART0, UART1] {
            assert_eq!(rd(&mut m, base + FR), 0x90);
            assert_eq!(rd(&mut m, base + CR), 0x300);
            for r in [IBRD, FBRD, LCR_H, IMSC, RIS] {
                assert_eq!(rd(&mut m, base + r), 0);
            }
        }
    }

    #[test]
    fn identification_registers() {
        // Datasheet 4.2.8: PeriphID 0x11 0x10 0x34 0x00, PCellID 0x0d 0xf0 0x05 0xb1.
        let mut m = Rp2040::new();
        let ids: Vec<u32> = [0xfe0, 0xfe4, 0xfe8, 0xfec, 0xff0, 0xff4, 0xff8, 0xffc].iter().map(|o| rd(&mut m, UART0 + o)).collect();
        assert_eq!(ids, [0x11, 0x10, 0x34, 0x00, 0x0d, 0xf0, 0x05, 0xb1]);
    }

    #[test]
    fn divisor_registers_are_sixteen_and_six_bits() {
        // IBRD is 16 bits, FBRD 6 bits (datasheet 4.2.8).
        let mut m = Rp2040::new();
        wr(&mut m, UART1 + IBRD, 0x12345);
        wr(&mut m, UART1 + FBRD, 0xff);
        assert_eq!(rd(&mut m, UART1 + IBRD), 0x2345);
        assert_eq!(rd(&mut m, UART1 + FBRD), 0x3f);
    }

    #[test]
    fn word_length_field_is_lcr_h_bits_6_5() {
        let mut m = Rp2040::new();
        for (v, bits) in [(0u32, 5), (1, 6), (2, 7), (3, 8)] {
            wr(&mut m, UART0 + LCR_H, v << 5);
            assert_eq!(m.periph.uart[0].word_length(), bits);
        }
    }

    #[test]
    fn written_bytes_go_to_the_host_in_order() {
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + DR, 0x141);
        ns(&mut m, 8.0);
        wr(&mut m, UART1 + DR, 0x42);
        let ev: Vec<(u8, u8, f64, f64)> = m.periph.uart_events.iter().map(|e| (e.uart, e.kind, e.value, e.nanos)).collect();
        assert_eq!(ev, [(0, 0, 0x41 as f64, 0.0), (1, 0, 0x42 as f64, 8.0)]);
    }

    #[test]
    fn received_bytes_come_out_in_order_with_flags_and_interrupt() {
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + IMSC, RXINTR);
        assert_eq!(m.cpu.pending_interrupts & (1 << 20), 0);
        m.uart_feed_byte(0, b'a');
        m.uart_feed_byte(0, b'b');
        assert_eq!(rd(&mut m, UART0 + FR) & (1 << 4), 0, "RXFE clear");
        assert_ne!(m.cpu.pending_interrupts & (1 << 20), 0, "IRQ 20 = UART0");
        assert_eq!(rd(&mut m, UART0 + MIS) & RXINTR, RXINTR);
        assert_eq!(rd(&mut m, UART0 + DR), b'a' as u32);
        assert_eq!(rd(&mut m, UART0 + RIS) & RXINTR, RXINTR, "one byte still waits");
        assert_eq!(rd(&mut m, UART0 + DR), b'b' as u32);
        assert_eq!(rd(&mut m, UART0 + RIS) & RXINTR, 0);
        assert_ne!(rd(&mut m, UART0 + FR) & (1 << 4), 0);
        assert_eq!(m.cpu.pending_interrupts & (1 << 20), 0, "only RXINTR is unmasked");
    }

    #[test]
    fn rx_fifo_holds_thirty_two_bytes() {
        // Datasheet: 32 x 8 receive FIFO; RXFF flags it full.
        let mut m = Rp2040::new();
        for i in 0..40u8 {
            m.uart_feed_byte(1, i);
        }
        assert_ne!(rd(&mut m, UART1 + FR) & (1 << 6), 0);
        for i in 0..32u32 {
            assert_eq!(rd(&mut m, UART1 + DR), i);
        }
        assert_ne!(rd(&mut m, UART1 + FR) & (1 << 4), 0);
    }

    #[test]
    fn enabling_the_uart_raises_the_tx_request_line() {
        let mut m = Rp2040::new();
        assert!(!m.periph.dma.dreq[dreq::UART0_TX as usize]);
        wr(&mut m, UART0 + CR, 0x301);
        assert!(m.periph.dma.dreq[dreq::UART0_TX as usize]);
        assert!(!m.periph.dma.dreq[dreq::UART1_TX as usize]);
        wr(&mut m, UART0 + CR, 0x300);
        assert!(!m.periph.dma.dreq[dreq::UART0_TX as usize]);
    }

    #[test]
    fn baud_changes_are_reported() {
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + IBRD, 67);
        wr(&mut m, UART0 + FBRD, 52);
        let rates: Vec<f64> = m.periph.uart_events.iter().filter(|e| e.kind == 1).map(|e| e.value).collect();
        // IBRD alone: 125 MHz / (67 * 16) = 116604.
        assert_eq!(rates, [116604.0, 115207.0]);
    }

    // ---- reference quirks -----------------------------------------------------------

    #[test]
    fn reference_quirk_tx_interrupt_is_raised_by_any_interrupt_check() {
        // Datasheet: TXRIS is set while the TX FIFO is at or below its trigger
        // level, and clears only when the level is crossed again. The
        // reference has no TX FIFO: every interrupt check sets it, so an ICR
        // write cannot clear it.
        let mut m = Rp2040::new();
        assert_eq!(rd(&mut m, UART0 + RIS), 0);
        wr(&mut m, UART0 + IMSC, TXINTR);
        assert_eq!(rd(&mut m, UART0 + RIS), TXINTR);
        wr(&mut m, UART0 + ICR, TXINTR);
        assert_eq!(rd(&mut m, UART0 + RIS), TXINTR /* datasheet: 0 */);
        assert_ne!(m.cpu.pending_interrupts & (1 << 20), 0);
    }

    #[test]
    fn reference_quirk_tx_fifo_is_always_empty() {
        // Datasheet: TXFE reflects the TX FIFO; a write makes TXFE clear until
        // the byte is sent. The reference sends at once and always reports empty.
        let mut m = Rp2040::new();
        wr(&mut m, UART0 + DR, 1);
        assert_ne!(rd(&mut m, UART0 + FR) & (1 << 7), 0);
        assert_eq!(rd(&mut m, UART0 + FR) & (1 << 5), 0, "TXFF never set");
    }

    #[test]
    fn reference_quirk_rx_dma_request_is_never_raised() {
        // Datasheet 4.2.3.4: with DMACR.RXDMAE the RX request follows the RX
        // FIFO. The reference has no UARTDMACR and never touches the RX line.
        let mut m = Rp2040::new();
        m.uart_feed_byte(0, 1);
        assert!(!m.periph.dma.dreq[dreq::UART0_RX as usize]);
        assert_eq!(rd(&mut m, UART0 + 0x48), 0xffff_ffff, "UARTDMACR is unimplemented");
    }

    #[test]
    fn reference_quirk_zero_divisor_reports_an_infinite_baud_rate() {
        let m = Rp2040::new();
        assert!(m.periph.uart[0].baud_rate().is_infinite());
    }
}

mod spi {
    use super::super::dma::dreq;
    use super::super::spi::SpiHost;
    use super::*;

    const SPI0: u32 = 0x4003_c000;
    const SPI1: u32 = 0x4004_0000;
    const CR0: u32 = 0x00;
    const CR1: u32 = 0x04;
    const DR: u32 = 0x08;
    const SR: u32 = 0x0c;
    const CPSR: u32 = 0x10;
    const IMSC: u32 = 0x14;
    const RIS: u32 = 0x18;
    const MIS: u32 = 0x1c;
    const ICR: u32 = 0x20;
    const BSY: u32 = 1 << 4;
    const RFF: u32 = 1 << 3;
    const RNE: u32 = 1 << 2;
    const TNF: u32 = 1 << 1;
    const TFE: u32 = 1 << 0;
    const SSE: u32 = 1 << 1;

    fn sent(m: &Rp2040) -> Vec<u32> {
        m.periph.spi_events.iter().map(|e| e.value).collect()
    }

    #[test]
    fn reset_values_and_ids() {
        // Datasheet 4.4.4: SSPSR resets to 0x3 (TFE, TNF); CR0, CR1, CPSR, IMSC are 0.
        let mut m = Rp2040::new();
        for b in [SPI0, SPI1] {
            assert_eq!(rd(&mut m, b + SR), TFE | TNF);
            for r in [CR0, CR1, CPSR, IMSC, MIS] {
                assert_eq!(rd(&mut m, b + r), 0);
            }
            let ids: Vec<u32> = [0xfe0, 0xfe4, 0xfe8, 0xfec, 0xff0, 0xff4, 0xff8, 0xffc].iter().map(|o| rd(&mut m, b + o)).collect();
            assert_eq!(ids, [0x22, 0x10, 0x34, 0x00, 0x0d, 0xf0, 0x05, 0xb1]);
        }
    }

    #[test]
    fn constructor_dma_lines() {
        // The constructor raises each TX line (FIFO not full) and lowers RX.
        let m = Rp2040::new();
        let d = &m.periph.dma.dreq;
        assert!(d[dreq::SPI0_TX as usize] && d[dreq::SPI1_TX as usize]);
        assert!(!d[dreq::SPI0_RX as usize] && !d[dreq::SPI1_RX as usize]);
    }

    #[test]
    fn a_word_goes_out_and_the_answer_comes_back() {
        let mut m = Rp2040::new();
        m.periph.spi[0].host = SpiHost::Respond(0x5a);
        wr(&mut m, SPI0 + CR0, 7); // DSS = 8 bits
        wr(&mut m, SPI0 + CR1, SSE);
        wr(&mut m, SPI0 + DR, 0xa5);
        assert_eq!(sent(&m), [0xa5]);
        assert_ne!(rd(&mut m, SPI0 + SR) & RNE, 0);
        assert_eq!(rd(&mut m, SPI0 + DR), 0xa5 ^ 0x5a);
        assert_eq!(rd(&mut m, SPI0 + SR) & RNE, 0);
        assert_eq!(rd(&mut m, SPI0 + DR), 0, "an empty FIFO reads zero");
    }

    #[test]
    fn the_default_answer_is_zero() {
        let mut m = Rp2040::new();
        wr(&mut m, SPI1 + CR1, SSE);
        wr(&mut m, SPI1 + DR, 0x12);
        assert_eq!(rd(&mut m, SPI1 + DR), 0);
    }

    #[test]
    fn data_size_masks_the_word() {
        // CR0.DSS = n - 1 for an n-bit frame.
        let mut m = Rp2040::new();
        wr(&mut m, SPI0 + CR0, 3);
        wr(&mut m, SPI0 + DR, 0xabcd);
        assert_eq!(sent(&m), [0xd]);
        wr(&mut m, SPI0 + CR0, 15);
        wr(&mut m, SPI0 + DR, 0xabcd);
        assert_eq!(sent(&m)[1], 0xabcd);
    }

    #[test]
    fn fifos_are_eight_deep_with_status_flags() {
        let mut m = Rp2040::new();
        m.periph.spi[0].host = SpiHost::Manual;
        wr(&mut m, SPI0 + CR0, 7);
        wr(&mut m, SPI0 + CR1, SSE);
        // The first word leaves for the shift register; eight more fill the FIFO.
        for i in 0..9 {
            assert_ne!(rd(&mut m, SPI0 + SR) & TNF, 0, "word {i}");
            wr(&mut m, SPI0 + DR, i);
        }
        let sr = rd(&mut m, SPI0 + SR);
        assert_eq!(sr & TNF, 0);
        assert_eq!(sr & TFE, 0);
        assert_ne!(sr & BSY, 0);
        assert!(!m.periph.dma.dreq[dreq::SPI0_TX as usize], "TX line follows 'not full'");
        // Answer them all: each answer starts the next word.
        for i in 0..9 {
            m.spi_complete_transmit(0, 100 + i);
        }
        assert_eq!(sent(&m), (0..9).collect::<Vec<u32>>());
        let sr = rd(&mut m, SPI0 + SR);
        assert_ne!(sr & TFE, 0);
        assert_ne!(sr & RFF, 0, "eight answers kept, the ninth overran");
        assert_ne!(rd(&mut m, SPI0 + RIS) & 1, 0, "RORRIS");
        assert!(m.periph.dma.dreq[dreq::SPI0_RX as usize]);
        assert_eq!(rd(&mut m, SPI0 + DR), 100);
        wr(&mut m, SPI0 + ICR, 3);
        assert_eq!(rd(&mut m, SPI0 + RIS) & 3, 0);
    }

    #[test]
    fn half_full_interrupts() {
        // Datasheet: RXRIS at RX FIFO >= half (4), TXRIS at TX FIFO <= half (4).
        let mut m = Rp2040::new();
        m.periph.spi[0].host = SpiHost::Manual;
        wr(&mut m, SPI0 + CR0, 7);
        wr(&mut m, SPI0 + IMSC, 0xc);
        for i in 0..3 {
            wr(&mut m, SPI0 + DR, i);
        }
        assert_ne!(rd(&mut m, SPI0 + RIS) & (1 << 3), 0);
        assert_ne!(m.cpu.pending_interrupts & (1 << 18), 0, "IRQ 18 = SPI0");
        for _ in 0..3 {
            m.spi_complete_transmit(0, 7);
        }
        assert_eq!(rd(&mut m, SPI0 + RIS) & (1 << 2), 0, "3 words received: below half");
        m.spi_complete_transmit(0, 7);
        wr(&mut m, SPI0 + DR, 1);
        m.spi_complete_transmit(0, 7);
        assert_ne!(rd(&mut m, SPI0 + RIS) & (1 << 2), 0, "4 words: RXRIS");
        assert_ne!(rd(&mut m, SPI0 + MIS) & (1 << 2), 0);
    }

    #[test]
    fn clock_rate_and_mode() {
        // Datasheet 4.4.3.1: FSSPCLKOUT = FSSPCLK / (CPSDVSR * (1 + SCR)).
        let mut m = Rp2040::new();
        wr(&mut m, SPI0 + CPSR, 2);
        wr(&mut m, SPI0 + CR0, 3 << 8);
        assert_eq!(m.periph.spi[0].clock_frequency(), 125e6 / 8.0);
        assert_eq!(rd(&mut m, SPI0 + CPSR), 2);
        wr(&mut m, SPI0 + CPSR, 7);
        assert_eq!(rd(&mut m, SPI0 + CPSR), 6, "CPSDVSR is even");
        for (cr0, mode) in [(0u32, 0), (1 << 7, 1), (1 << 6, 3), (3 << 6, 2)] {
            wr(&mut m, SPI0 + CR0, cr0);
            assert_eq!(m.periph.spi[0].spi_mode(), mode, "cr0 {cr0:#x}");
        }
        assert!(m.periph.spi[0].master_mode());
        wr(&mut m, SPI0 + CR0, 1 << 2);
        assert!(!m.periph.spi[0].master_mode());
    }

    #[test]
    fn reference_quirk_transmit_ignores_sse() {
        // Datasheet: nothing is transmitted while SSPCR1.SSE is 0.
        let mut m = Rp2040::new();
        wr(&mut m, SPI0 + CR0, 7);
        wr(&mut m, SPI0 + DR, 0x33);
        assert_eq!(sent(&m), [0x33] /* datasheet: [] */);
    }

    #[test]
    fn reference_quirk_write_to_a_full_tx_fifo_is_dropped() {
        // Datasheet: the FIFO's last entry is overwritten / the write is ignored
        // with no flag; the reference ignores it too, which is what this pins:
        // eleven writes, nine words leave (one in flight plus eight queued).
        let mut m = Rp2040::new();
        m.periph.spi[1].host = SpiHost::Manual;
        wr(&mut m, SPI1 + CR0, 7);
        for i in 0..11 {
            wr(&mut m, SPI1 + DR, i);
        }
        for _ in 0..11 {
            m.spi_complete_transmit(1, 0);
        }
        assert_eq!(sent(&m), (0..9).collect::<Vec<u32>>());
    }

    #[test]
    fn reference_quirk_tx_interrupt_starts_clear() {
        // Datasheet 4.4.4: SSPRIS resets to 0x8 (TXRIS: the empty FIFO).
        let mut m = Rp2040::new();
        assert_eq!(rd(&mut m, SPI0 + RIS), 0 /* datasheet: 0x8 */);
    }

    #[test]
    fn reference_quirk_imsc_is_not_masked() {
        // Datasheet: SSPIMSC has four bits.
        let mut m = Rp2040::new();
        wr(&mut m, SPI0 + IMSC, 0xffff);
        assert_eq!(rd(&mut m, SPI0 + IMSC), 0xffff /* datasheet: 0xf */);
    }
}
