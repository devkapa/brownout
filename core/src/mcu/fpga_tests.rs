//! The fabric on the reference's own fixtures: each `.image` beside the
//! netlists in test/sim/engine/__fixtures__/fpga is `Ice40.image()` of that
//! netlist (ints on the first line, floats on the second), so these hold the
//! port to the same pin behaviour test/sim/engine/fpga.test.ts holds the
//! reference to.

use super::{Fabric, Oscillates};

fn load(text: &str) -> Fabric {
    let mut lines = text.lines();
    let ints: Vec<i32> = lines.next().unwrap().split_whitespace().map(|v| v.parse().unwrap()).collect();
    let floats: Vec<f64> = lines.next().unwrap_or("").split_whitespace().map(|v| v.parse().unwrap()).collect();
    Fabric::from_image(&ints, &floats).expect("the fixture settles")
}

const BLINKY: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/blinky.image");
const BUTTON: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/button.image");
const DIVIDER: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/divider.image");
const PCF_LED: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/pcf_led.image");

fn bit(v: Option<bool>) -> Option<u8> {
    v.map(|b| b as u8)
}

#[test]
fn blinky_counts_the_hfosc_into_the_rgb_pads() {
    let mut f = load(BLINKY);
    // Power-up: count 0, every RGB pad open, gpio_3 = count[3] ^ gpio_2.
    assert_eq!([f.pin_drive(39), f.pin_drive(40), f.pin_drive(41)], [None, None, None]);
    f.set_pin(2, Some(false)).unwrap();
    assert_eq!(bit(f.pin_drive(3)), Some(0));
    // 0.75 s of 12 MHz: 9,000,000 rising edges, 0x895440: bit 23 (green) set, 22 and 21 clear.
    f.advance(0.75).unwrap();
    let count = 9_000_000u32;
    assert_eq!(f.pin_drive(39), Some(false));
    assert_eq!(f.pin_drive(40), None);
    assert_eq!(f.pin_drive(41), None);
    assert_eq!(bit(f.pin_drive(3)), Some(((count >> 3) & 1) as u8));
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!(bit(f.pin_drive(3)), Some((((count >> 3) & 1) ^ 1) as u8));
}

#[test]
fn button_reads_through_its_pull_up_and_drives_only_while_enabled() {
    let mut f = load(BUTTON);
    f.set_pin(2, None).unwrap();
    assert_eq!(bit(f.pin_drive(4)), Some(0));
    f.set_pin(2, Some(false)).unwrap();
    assert_eq!(bit(f.pin_drive(4)), Some(1));
    f.set_pin(9, Some(true)).unwrap();
    assert_eq!(bit(f.pin_drive(6)), Some(1));
    f.set_pin(9, Some(false)).unwrap();
    assert_eq!(f.pin_drive(6), None);
    // A pin the design does not use reads as undriven, and setting it is a no-op.
    assert_eq!(f.pin_drive(12), None);
    f.set_pin(12, Some(true)).unwrap();
}

#[test]
fn divider_ripples_down_and_holds_at_zero_in_reset() {
    let mut f = load(DIVIDER);
    f.set_pin(2, Some(false)).unwrap();
    let period = 1.0 / 12e6;
    f.advance(period * 1.25).unwrap();
    for k in 1..=40i32 {
        assert_eq!(bit(f.pin_drive(4)), Some((k & 1) as u8), "t0 after {k}");
        assert_eq!(bit(f.pin_drive(3)), Some(((-k & 15) >> 3) as u8), "t3 after {k}");
        assert_eq!(bit(f.pin_drive(6)), Some((k & 1) as u8), "n after {k}");
        f.advance(period).unwrap();
    }
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!([bit(f.pin_drive(3)), bit(f.pin_drive(4))], [Some(0), Some(0)]);
    f.advance(period * 10.0).unwrap();
    assert_eq!([bit(f.pin_drive(3)), bit(f.pin_drive(4))], [Some(0), Some(0)]);
}

#[test]
fn pcf_led_counts_only_while_enabled_from_its_initial_value() {
    let mut f = load(PCF_LED);
    let leds = |f: &Fabric| [3, 4, 44, 6].iter().enumerate().fold(0u32, |v, (i, &p)| v | (f.pin_drive(p).unwrap_or(false) as u32) << i);
    assert_eq!(leds(&f), 5);
    f.set_pin(2, Some(false)).unwrap();
    f.advance(10.2e-6).unwrap();
    assert_eq!(leds(&f), 5);
    f.set_pin(2, Some(true)).unwrap();
    f.advance(13e-6).unwrap();
    assert_eq!(leds(&f), (5 + 13) & 15);
}

#[test]
fn a_flip_flop_pair_clocking_itself_on_both_edges_never_settles() {
    // q is a rising- and a falling-edge flip-flop both clocked by and fed !q:
    // every change of q is a new edge of its own clock.
    #[rustfmt::skip]
    let ints = [
        4, 1, 1, 2, 0, 0,
        0, 3, 2, 0, 0, 0, // LUT: net 3 = !net 2
        1,                // LUT_INIT
        0,                // level
        3, 2, 3, 0, 1, -1, 0, 0,
        3, 2, 3, 1, 1, -1, 0, 0,
    ];
    assert_eq!(Fabric::from_image(&ints, &[]).err(), Some(Oscillates));
}
