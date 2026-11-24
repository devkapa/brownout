//! The fabric on the reference's own fixtures: each `.image` beside the
//! netlists in test/sim/engine/__fixtures__/fpga is `Ice40.image()` of that
//! netlist (ints on the first line, floats on the second), so these hold the
//! port to the same pin behaviour test/sim/engine/fpga.test.ts holds the
//! reference to.

use super::{AdvanceError, Fabric, Oscillates};

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
const TBUF: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/tbuf.image");
const CONSTANT: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/constant.image");
const GATED: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/gated.image");
const READBACK: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/readback.image");
const RIPPLE70: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/ripple70.image");
const RING: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/ring.image");
const EDGE: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/edge.image");
const DFFS: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/dffs.image");
const RGB_IO: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/rgb_io.image");
const RGB_SINK: &str = include_str!("../../../test/sim/engine/__fixtures__/fpga/rgb_sink.image");

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
fn a_tristate_port_is_released_while_disabled_and_reads_the_pin_back() {
    let mut f = load(TBUF);
    f.set_pin(9, Some(false)).unwrap();
    f.set_pin(2, Some(false)).unwrap();
    assert_eq!(f.pin_drive(6), None);
    f.set_pin(6, Some(true)).unwrap();
    assert_eq!(f.pin_drive(3), Some(true));
    f.set_pin(6, Some(false)).unwrap();
    assert_eq!(f.pin_drive(3), Some(false));
    // Enabled, the design's level is on the pin and is what it reads back.
    f.set_pin(6, Some(true)).unwrap();
    f.set_pin(9, Some(true)).unwrap();
    assert_eq!([f.pin_drive(6), f.pin_drive(3)], [Some(false), Some(false)]);
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!([f.pin_drive(6), f.pin_drive(3)], [Some(true), Some(true)]);
}

#[test]
fn constant_outputs_drive_their_pins() {
    let f = load(CONSTANT);
    assert_eq!([f.pin_drive(3), f.pin_drive(4), f.pin_drive(16)], [Some(true), Some(false), Some(true)]);
}

#[test]
fn logic_gates_the_oscillator_and_the_rgb_driver() {
    let mut f = load(GATED);
    f.set_pin(2, Some(false)).unwrap();
    f.set_pin(4, Some(false)).unwrap();
    f.advance(1.04e-6).unwrap();
    assert_eq!(f.pin_drive(3), Some(false));
    assert_eq!(f.pin_drive(39), None);
    f.set_pin(4, Some(true)).unwrap();
    assert_eq!(f.pin_drive(39), Some(false));
    // Blue has no current set, so it stays dark.
    assert_eq!(f.pin_drive(40), None);
    f.set_pin(2, Some(true)).unwrap();
    f.advance(0.5e-6).unwrap();
    assert_eq!(f.pin_drive(3), Some(true));
    f.set_pin(2, Some(false)).unwrap();
    f.advance(10e-6).unwrap();
    assert_eq!(f.pin_drive(3), Some(true));
}

#[test]
fn an_open_drain_pin_reads_back_and_power_up_is_no_edge() {
    let mut f = load(READBACK);
    assert_eq!(f.pin_drive(4), Some(false));
    f.set_pin(9, Some(false)).unwrap();
    f.set_pin(6, None).unwrap();
    assert_eq!([f.pin_drive(6), f.pin_drive(3)], [None, Some(true)]);
    f.set_pin(9, Some(true)).unwrap();
    assert_eq!([f.pin_drive(6), f.pin_drive(3)], [Some(false), Some(false)]);
    f.set_pin(2, Some(false)).unwrap();
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!(f.pin_drive(4), Some(true));
}

#[test]
fn a_70_stage_ripple_settles_and_the_board_clock_pin_ignores_the_circuit() {
    let mut f = load(RIPPLE70);
    f.advance(1.02e-6).unwrap();
    assert_eq!(f.edge_count, 24);
    f.set_pin(20, Some(true)).unwrap();
    f.set_pin(20, Some(false)).unwrap();
    f.advance(1e-6).unwrap();
    assert_eq!(f.edge_count, 48);
    assert_eq!(f.pin_drive(3), Some(true));
}

#[test]
fn flip_flops_clocking_themselves_once_enabled_never_settle() {
    // q1 rises and q2 falls on q1 ^ q2 ^ gpio_2: once gpio_2 rises every
    // toggle is the next edge of the same instant.
    let mut f = load(RING);
    assert_eq!(f.set_pin(2, Some(false)), Ok(()));
    assert_eq!(f.set_pin(2, Some(true)), Err(Oscillates));
}

#[test]
fn a_level_taken_while_configuring_is_no_edge() {
    let mut f = load(EDGE);
    f.preset_pin(2, Some(true)).unwrap();
    assert_eq!(f.pin_drive(3), Some(false));
    f.set_pin(2, Some(false)).unwrap();
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!(f.pin_drive(3), Some(true));
}

#[test]
fn an_asynchronous_set_tied_high_holds_its_flip_flop() {
    let mut f = load(DFFS);
    assert_eq!(f.pin_drive(3), Some(true));
    f.set_pin(2, Some(true)).unwrap();
    f.set_pin(2, Some(false)).unwrap();
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!(f.pin_drive(3), Some(true));
}

#[test]
fn an_rgb_pad_as_plain_output_is_open_drain() {
    let mut f = load(RGB_IO);
    f.set_pin(2, Some(true)).unwrap();
    assert_eq!(f.pin_drive(41), None);
    f.set_pin(2, Some(false)).unwrap();
    assert_eq!(f.pin_drive(41), Some(false));
}

#[test]
fn rgb_pads_sink_their_set_current() {
    let mut f = load(RGB_SINK);
    f.set_pin(2, Some(false)).unwrap();
    assert_eq!([f.sink_current(39), f.sink_current(41)], [24e-3, 0.0]);
    f.set_pin(2, Some(true)).unwrap();
    assert!((f.sink_current(41) - 8e-3).abs() < 1e-12);
    assert_eq!(f.sink_current(2), 0.0);
}

#[test]
fn a_step_that_is_not_a_finite_time_is_refused() {
    let mut f = load(BLINKY);
    for t in [f64::NAN, f64::INFINITY, -1.0] {
        assert_eq!(f.advance(t), Err(AdvanceError::BadTime));
    }
}

#[test]
fn a_step_stops_at_its_edge_budget_and_keeps_the_clock_phase() {
    let mut f = load(BLINKY);
    assert_eq!(f.advance_within(0.0100001, 1000), Ok(false));
    assert_eq!(f.edge_count, 1000);
    assert_eq!(f.time, 0.0100001);
    assert_eq!(f.advance_within(1.01e-6, u64::MAX), Ok(true));
    assert_eq!(f.edge_count, 1024);
}
