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
