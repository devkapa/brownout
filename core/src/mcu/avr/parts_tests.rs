//! The board's parts (parts.rs, ssd1306.rs, bmp280.rs, dht.rs): the models'
//! own cases, and real Adafruit sketches (built with devolt's in-browser
//! avr-gcc; the .ino files sit beside the hex) reading them through the
//! timed TWI and a pin target, as brownout's engine tests read the
//! reference models.

use super::bmp280::{self, Bmp280};
use super::cpu::Cpu;
use super::dht::{self, DhtSensor};
use super::parts::{js_round, I2cPart, PinSlot};
use super::periph_tests::board;
use super::ssd1306::Ssd1306;

const ADAFRUIT_SSD1306: &str = include_str!("../../../../test/sim/engine/__fixtures__/adafruit_ssd1306.hex");
const ADAFRUIT_BMP280: &str = include_str!("../../../../test/sim/engine/__fixtures__/adafruit_bmp280.hex");
const ADAFRUIT_DHT22: &str = include_str!("../../../../test/sim/engine/__fixtures__/adafruit_dht22.hex");
const ADAFRUIT_DHT11: &str = include_str!("../../../../test/sim/engine/__fixtures__/adafruit_dht11.hex");

fn run_seconds(cpu: &mut Cpu, seconds: f64) {
    let end = cpu.cycles + (seconds * 16e6) as u64;
    while cpu.cycles < end {
        super::instr::avr_instruction(cpu);
        cpu.tick();
    }
}

fn serial_text(cpu: &mut Cpu) -> String {
    String::from_utf8_lossy(&cpu.serial_drain()).into_owned()
}

fn oled() -> Ssd1306 {
    let mut d = Ssd1306::new(0x3c, 64);
    d.powered = true;
    d
}

#[test]
fn js_round_takes_halves_up_as_math_round_does() {
    assert_eq!(js_round(2.5), 3.0);
    assert_eq!(js_round(-2.5), -2.0);
    assert_eq!(js_round(-123.5), -123.0);
    assert_eq!(js_round(-0.4), -0.0);
    assert_eq!(js_round(7.49), 7.0);
}

#[test]
fn ssd1306_writes_gddram_in_page_and_horizontal_addressing() {
    let mut d = oled();
    d.start();
    // Commands: horizontal addressing, columns 126-127, pages 6-7.
    for b in [0x00, 0x20, 0x00, 0x21, 126, 127, 0x22, 6, 7] {
        assert!(d.write(b));
    }
    d.stop();
    d.start();
    d.write(0x40);
    for v in 1..=5u8 {
        d.write(v);
    }
    // Four fill the window; the fifth wraps to its start.
    assert_eq!([d.ram[6 * 128 + 126], d.ram[6 * 128 + 127], d.ram[7 * 128 + 126], d.ram[7 * 128 + 127]], [5, 2, 3, 4]);
    assert_eq!((d.col, d.page), (127, 6));
    // Status reads bit 6 while the display is off.
    assert_eq!(d.read(), 0x40);
}

#[test]
fn ssd1306_image_is_dark_until_the_display_and_charge_pump_are_on() {
    let mut d = oled();
    d.ram[0] = 0x01;
    assert!(!d.image().on);
    d.start();
    for b in [0x00, 0x8d, 0x14, 0xaf, 0xa1, 0xc8] {
        d.write(b);
    }
    let img = d.image();
    assert!(img.on);
    // Column 0 of page 0, bit 0, is the top-left pixel with the remaps set.
    assert_eq!(img.pixels[0], 0x80);
    assert_eq!(img.pixels.iter().map(|b| b.count_ones()).sum::<u32>(), 1);
}

#[test]
fn bmp280_raw_readings_compensate_back_to_the_ambient_values() {
    let t = bmp280::raw_temperature(23.45);
    let (t_fine, centi) = bmp280::temperature(t);
    assert_eq!(centi, 2345);
    let p = bmp280::raw_pressure(98765.0, t_fine);
    assert!((bmp280::pressure(p, t_fine) - 98765 * 256).abs() <= 128);
    // Negative temperatures round as Math.round does.
    assert_eq!(bmp280::temperature(bmp280::raw_temperature(-12.345)).1, -1234);
}

#[test]
fn bmp280_forced_conversion_takes_its_measurement_time_on_the_board_clock() {
    let mut b = Bmp280::new(0x77);
    b.powered = true;
    b.clocked = true;
    b.set_ambient(30.0, 100000.0, 0.0);
    b.start(0.0);
    // ctrl_meas: x1 temperature, x1 pressure, forced.
    b.write(0xf4, 0.0);
    b.write(0x25, 0.0);
    b.stop(0.0);
    let read_status = |b: &mut Bmp280, t: f64| {
        b.start(t);
        b.write(0xf3, t);
        b.stop(t);
        b.start(t);
        let s = b.read(t);
        b.stop(t);
        s
    };
    assert_eq!(read_status(&mut b, 0.001), 0x08, "measuring during the 5.5 ms conversion");
    assert_eq!(read_status(&mut b, 0.006), 0x00);
    assert_eq!(b.mode(), 0, "forced mode returns to sleep");
    assert_ne!(b.adc_t, 0x80000);
}

#[test]
fn dht_frames_carry_tenths_and_the_sign_bit() {
    assert_eq!(dht::frame(22, 63.4, -7.8), [0x02, 0x7a, 0x80, 0x4e, 0x4a]);
    assert_eq!(dht::frame(11, 42.0, 23.4), [42, 0, 23, 4, 69]);
    // Clamped to each part's range.
    assert_eq!(dht::frame(11, 99.0, 60.0)[..4], [90, 0, 50, 0]);
}

#[test]
fn a_dht_answers_only_a_long_enough_start_after_its_first_second() {
    let mut s = DhtSensor::new(22);
    s.powered = true;
    s.set_ambient(50.0, 20.0);
    let hz = 16e6;
    s.board_drive(Some(false), 0, hz);
    assert!(matches!(s.board_drive(None, 16_000, hz), dht::DhtAction::None), "inside the power-up second");
    s.powered_for = 2.0;
    s.board_drive(Some(false), 100_000, hz);
    assert!(matches!(s.board_drive(None, 100_000 + 799 * 16, hz), dht::DhtAction::None), "799 us is too short");
    s.board_drive(Some(false), 200_000, hz);
    let dht::DhtAction::Answer(edges) = s.board_drive(None, 200_000 + 1000 * 16, hz) else { panic!("no answer") };
    assert_eq!(edges.len(), dht::ANSWER_EDGES);
    assert_eq!(edges[0], (30 * 16, true));
    assert_eq!(edges[1], (110 * 16, false));
}

#[test]
fn a_pin_target_that_goes_away_lets_go_of_the_pin() {
    let mut cpu = board(ADAFRUIT_DHT22);
    let mut dht = DhtSensor::new(22);
    dht.powered = true;
    cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht }]);
    cpu.set_input_bit(2, 2, true);
    cpu.p.parts.target_low = 1 << 2;
    cpu.set_input_bit(2, 2, true);
    assert_eq!(cpu.p.ports[2].pin_value & 0x04, 0, "held low by the part");
    cpu.set_pin_targets(Vec::new());
    assert_eq!(cpu.p.parts.target_low, 0);
    assert_eq!(cpu.p.ports[2].pin_value & 0x04, 0x04, "back to the circuit's level");
}

#[test]
fn an_adafruit_ssd1306_sketch_draws_its_frame() {
    let mut cpu = board(ADAFRUIT_SSD1306);
    cpu.p.parts.i2c.push(I2cPart::Ssd1306(oled()));
    run_seconds(&mut cpu, 0.4);
    let I2cPart::Ssd1306(d) = &cpu.p.parts.i2c[0] else { unreachable!() };
    let img = d.image();
    assert!(img.on);
    let lit = |x: usize, y: usize| (img.pixels[y * 16 + (x >> 3)] >> (7 - (x & 7))) & 1;
    for (x, y) in [(0, 0), (127, 0), (0, 63), (127, 63), (64, 0), (0, 32), (127, 32), (64, 63)] {
        assert_eq!(lit(x, y), 1, "border {x},{y}");
    }
    for y in 40..56 {
        for x in 100..120 {
            assert_eq!(lit(x, y), 1, "block {x},{y}");
        }
    }
    assert_eq!(lit(64, 32), 1);
    assert_eq!(lit(63, 32) + lit(65, 32) + lit(64, 31) + lit(64, 33), 0);
    let mut text = 0;
    for y in 4..12 {
        for x in 4..4 + 7 * 6 {
            text += lit(x, y) as u32;
        }
    }
    assert!(text > 40);
    let total: u32 = img.pixels.iter().map(|b| b.count_ones()).sum();
    assert_eq!(total, 2 * 128 + 2 * 62 + 20 * 16 + 1 + text);
}

#[test]
fn an_unpowered_ssd1306_leaves_the_bus_empty() {
    let mut cpu = board(ADAFRUIT_SSD1306);
    let mut d = oled();
    d.powered = false;
    cpu.p.parts.i2c.push(I2cPart::Ssd1306(d));
    run_seconds(&mut cpu, 0.1);
    let I2cPart::Ssd1306(d) = &cpu.p.parts.i2c[0] else { unreachable!() };
    assert!(!d.image().on);
}

#[test]
fn an_adafruit_bmp280_sketch_reads_the_ambient_values() {
    let mut cpu = board(ADAFRUIT_BMP280);
    let mut b = Bmp280::new(0x77);
    b.powered = true;
    b.set_ambient(23.45, 98765.0, 0.0);
    cpu.p.parts.i2c.push(I2cPart::Bmp280(b));
    run_seconds(&mut cpu, 0.5);
    let text = serial_text(&mut cpu);
    let readings: Vec<(f64, f64)> = text
        .lines()
        .filter_map(|l| {
            let l = l.trim().strip_prefix("T=")?;
            let (t, p) = l.split_once(" P=")?;
            Some((t.parse().ok()?, p.parse().ok()?))
        })
        .collect();
    assert!(readings.len() >= 3, "{text:?}");
    for (t, p) in readings {
        assert_eq!(t, 23.45);
        assert!((p - 98765.0).abs() <= 0.5, "{p}");
    }
}

fn dht_readings(variant: u8, humidity: f64, celsius: f64, hex: &str, seconds: f64) -> Vec<(f64, f64)> {
    let mut cpu = board(hex);
    let mut s = DhtSensor::new(variant);
    s.powered = true;
    s.powered_for = 2.0;
    s.set_ambient(humidity, celsius);
    cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht: s }]);
    // The module's pull-up holds DATA high.
    cpu.set_input_bit(2, 2, true);
    run_seconds(&mut cpu, seconds);
    serial_text(&mut cpu)
        .lines()
        .filter_map(|l| {
            let l = l.trim().strip_prefix("H=")?;
            let (h, t) = l.split_once(" T=")?;
            Some((h.parse().ok()?, t.parse().ok()?))
        })
        .collect()
}

#[test]
fn an_adafruit_dht22_sketch_reads_humidity_and_a_negative_temperature() {
    assert_eq!(dht_readings(22, 63.4, -7.8, ADAFRUIT_DHT22, 4.5), vec![(63.4, -7.8), (63.4, -7.8)]);
}

#[test]
fn an_adafruit_dht11_sketch_reads_whole_humidity_and_tenths() {
    assert_eq!(dht_readings(11, 42.0, 23.4, ADAFRUIT_DHT11, 2.5), vec![(42.0, 23.4)]);
}
