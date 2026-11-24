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

// ---- review cases: parts that change under a running board ----------------

const SSD1306_HELLO: &str = include_str!("../../../../test/sim/engine/__fixtures__/ssd1306_hello.hex");

fn step(cpu: &mut Cpu) {
    super::instr::avr_instruction(cpu);
    cpu.tick();
}

#[test]
fn a_start_from_before_a_board_rebuild_gets_no_answer() {
    // The sketch held the line low late in a long run; the rebuilt board's
    // clock starts over below that cycle. The reference's signed difference
    // is negative, so too short: no answer.
    let mut s = DhtSensor::new(22);
    s.powered = true;
    s.powered_for = 2.0;
    s.board_drive(Some(false), 50_000_000, 16e6);
    assert!(matches!(s.board_drive(None, 100, 16e6), dht::DhtAction::None));
}

#[test]
fn an_answer_stops_when_its_part_is_taken_off_and_never_reaches_another_pin() {
    let mut cpu = board(ADAFRUIT_DHT22);
    let sensor = |v| {
        let mut s = DhtSensor::new(v);
        s.powered = true;
        s.powered_for = 2.0;
        s.set_ambient(50.0, 20.0);
        s
    };
    cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht: sensor(22) }, PinSlot { port: 2, bit: 4, dht: sensor(22) }]);
    cpu.set_input_bit(2, 2, true);
    cpu.set_input_bit(2, 4, true);
    // A 2 ms start on d2, then the release: the d2 part answers.
    let dht = &mut cpu.p.parts.pins[0].dht;
    dht.board_drive(Some(false), 0, 16e6);
    let dht::DhtAction::Answer(edges) = dht.board_drive(None, 32_000, 16e6) else { panic!("no answer") };
    let id = cpu.p.parts.begin_answer(0);
    for (i, (cycles, _)) in edges.iter().enumerate() {
        cpu.add_clock_event_arg(super::parts::EV_PIN_TARGET, super::parts::answer_edge_arg(i, 2, id), *cycles);
    }
    // The d2 part is taken off: the d4 part moves into the first slot.
    let d4 = cpu.p.parts.pins[1].clone();
    cpu.set_pin_targets(vec![d4]);
    let end = cpu.cycles + 16 * 6000;
    while cpu.cycles < end {
        step(&mut cpu);
        assert_eq!(cpu.p.parts.target_low, 0, "no pin is pulled");
    }
    assert!(!cpu.p.parts.pins[0].dht.answering);
}

#[test]
fn two_parts_on_one_pin_keep_the_last_as_a_map_does() {
    let mut cpu = board(ADAFRUIT_DHT22);
    cpu.set_pin_targets(vec![
        PinSlot { port: 2, bit: 2, dht: DhtSensor::new(22) },
        PinSlot { port: 2, bit: 2, dht: DhtSensor::new(11) },
    ]);
    assert_eq!(cpu.p.parts.pins.len(), 1);
    assert_eq!(cpu.p.parts.pins[0].dht.variant, 11);
}

#[test]
fn an_i2c_part_that_loses_power_mid_transaction_nacks_the_rest() {
    let mut cpu = board(SSD1306_HELLO);
    cpu.p.parts.i2c.push(I2cPart::Ssd1306(oled()));
    let mut seen = Vec::new();
    let acked = |e: &super::bus::BusEvent| {
        matches!(e, super::bus::BusEvent::I2c { kind: super::bus::I2cKind::Address { ack: true, .. }, .. })
    };
    while !seen.iter().any(acked) && cpu.cycles < 1_000_000 {
        step(&mut cpu);
        seen.extend(cpu.bus_drain());
    }
    assert!(seen.iter().any(acked));
    let I2cPart::Ssd1306(d) = &mut cpu.p.parts.i2c[0] else { unreachable!() };
    d.powered = false;
    let mut after = Vec::new();
    let end = cpu.cycles + 32_000;
    while cpu.cycles < end {
        step(&mut cpu);
        after.extend(cpu.bus_drain());
    }
    let writes: Vec<bool> = after
        .iter()
        .take_while(|e| !matches!(e, super::bus::BusEvent::I2c { kind: super::bus::I2cKind::Stop, .. }))
        .filter_map(|e| match e {
            super::bus::BusEvent::I2c { kind: super::bus::I2cKind::Write { ack, .. }, .. } => Some(*ack),
            _ => None,
        })
        .collect();
    assert!(!writes.is_empty());
    assert!(writes.iter().all(|a| !a), "{writes:?}");
    // Taking a part off mid-transaction cannot reach a stale index either.
    cpu.p.parts.i2c.clear();
    let end = cpu.cycles + 32_000;
    while cpu.cycles < end {
        step(&mut cpu);
    }
}

// ---- more of the reference models' own cases -----------------------------

fn bmp_at(b: &mut Bmp280, t: f64, f: impl FnOnce(&mut Bmp280, f64)) {
    f(b, t)
}

fn write_regs(b: &mut Bmp280, t: f64, bytes: &[u8]) {
    b.start(t);
    for &v in bytes {
        assert!(b.write(v, t));
    }
    b.stop(t);
}

fn read_regs(b: &mut Bmp280, t: f64, register: u8, n: usize) -> Vec<u8> {
    b.start(t);
    b.write(register, t);
    b.stop(t);
    b.start(t);
    let out = (0..n).map(|_| b.read(t)).collect();
    b.stop(t);
    out
}

fn compensated(b: &mut Bmp280, t: f64) -> (f64, f64) {
    let r = read_regs(b, t, 0xf7, 6);
    let adc_p = (r[0] as i64) << 12 | (r[1] as i64) << 4 | (r[2] as i64) >> 4;
    let adc_t = (r[3] as i64) << 12 | (r[4] as i64) << 4 | (r[5] as i64) >> 4;
    let (t_fine, centi) = bmp280::temperature(adc_t);
    (centi as f64 / 100.0, bmp280::pressure(adc_p, t_fine) as f64 / 256.0)
}

fn bmp() -> Bmp280 {
    let mut b = Bmp280::new(0x77);
    b.powered = true;
    b.clocked = true;
    b
}

#[test]
fn bmp280_compensates_the_datasheet_worked_example() {
    let (t_fine, centi) = bmp280::temperature(519888);
    assert_eq!(centi, 2508);
    // The datasheet's figure is from its floating-point formula; the 64-bit one
    // lands within half a pascal of it.
    assert!((bmp280::pressure(415148, t_fine) as f64 / 256.0 - 100653.27).abs() < 0.5);
}

#[test]
fn bmp280_repeats_conversions_in_normal_mode() {
    let mut b = bmp();
    b.set_ambient(20.0, 101325.0, 0.0);
    write_regs(&mut b, 0.0, &[0xf5, 0b001_000_00, 0xf4, 0b001_001_11]);
    assert!((compensated(&mut b, 0.006).0 - 20.0).abs() < 0.05);
    b.set_ambient(30.0, 101325.0, 0.006);
    assert!((compensated(&mut b, 0.07).0 - 20.0).abs() < 0.05);
    assert!((compensated(&mut b, 0.074).0 - 30.0).abs() < 0.05);
    assert_eq!(read_regs(&mut b, 0.074, 0xf4, 1)[0] & 3, 3);
}

#[test]
fn bmp280_smooths_a_step_with_the_iir_filter() {
    let mut b = bmp();
    b.set_ambient(25.0, 100000.0, 0.0);
    write_regs(&mut b, 0.0, &[0xf5, 0b000_010_00, 0xf4, 0b001_101_11]);
    let period = 0.0005 + (1.0 + 2.0 + 32.5) / 1000.0;
    assert!((compensated(&mut b, period * 3.0).1 - 100000.0).abs() < 0.5);
    b.set_ambient(25.0, 101000.0, period * 3.0);
    let first = compensated(&mut b, period * 4.0 + 0.0001).1;
    assert!(first > 100200.0 && first < 100300.0, "{first}");
    assert!((compensated(&mut b, period * 60.0).1 - 101000.0).abs() < 0.5);
}

#[test]
fn bmp280_keeps_a_burst_read_to_one_conversion() {
    let mut b = bmp();
    b.set_ambient(10.0, 101325.0, 0.0);
    write_regs(&mut b, 0.0, &[0xf5, 0, 0xf4, 0b001_001_11]);
    b.set_ambient(40.0, 101325.0, 0.0056);
    b.start(0.0056);
    b.write(0xf7, 0.0056);
    b.stop(0.0056);
    b.start(0.0056);
    for _ in 0..3 {
        b.read(0.0056);
    }
    let tail: Vec<u8> = (0..3).map(|_| b.read(0.0125)).collect();
    b.stop(0.0125);
    let adc_t = (tail[0] as i64) << 12 | (tail[1] as i64) << 4 | (tail[2] as i64) >> 4;
    assert!((bmp280::temperature(adc_t).1 as f64 / 100.0 - 10.0).abs() < 0.05);
}

#[test]
fn bmp280_soft_reset_takes_only_b6h() {
    let mut b = bmp();
    write_regs(&mut b, 0.0, &[0xf5, 0xa0, 0xf4, 0x27]);
    write_regs(&mut b, 0.1, &[0xe0, 0x12]);
    assert_eq!(read_regs(&mut b, 0.1, 0xf4, 2), vec![0x27, 0xa0]);
    write_regs(&mut b, 0.1, &[0xe0, 0xb6]);
    assert_eq!(read_regs(&mut b, 0.1, 0xf4, 2), vec![0, 0]);
    assert_eq!((b.adc_t, b.adc_p), (0x80000, 0x80000));
}

#[test]
fn bmp280_finishes_a_conversion_a_board_started_before_it_reset() {
    let mut b = bmp();
    bmp_at(&mut b, 5.0, |b, t| write_regs(b, t, &[0xf4, 0b001_001_01]));
    let _ = read_regs(&mut b, 0.001, 0xf7, 6);
    assert_ne!(b.adc_t, 0x80000);
}

#[test]
fn ssd1306_takes_a_byte_with_co_set_as_one_command() {
    let mut d = oled();
    d.start();
    for b in [0x80, 0xa7, 0x40, 0x01] {
        d.write(b);
    }
    assert!(d.inverse);
    assert!(d.data_mode);
    assert_eq!(d.ram[0], 0x01);
}

#[test]
fn ssd1306_page_addressing_keeps_the_pointer_on_its_page() {
    let mut d = oled();
    d.start();
    for b in [0x00, 0x20, 0x02, 0xb3, 0x05, 0x12] {
        d.write(b);
    }
    d.stop();
    d.start();
    d.write(0x40);
    d.write(0xff);
    d.write(0x01);
    assert_eq!((d.ram[3 * 128 + 0x25], d.ram[3 * 128 + 0x26]), (0xff, 0x01));
    assert_eq!((d.page, d.col), (3, 0x27));
}

#[test]
fn a_dht_reports_the_measurement_made_after_the_previous_reading() {
    let mut s = DhtSensor::new(22);
    s.powered = true;
    s.powered_for = 2.0;
    s.set_ambient(40.0, 20.0);
    // First answer: the power-up measurement. The last edge measures anew.
    s.board_drive(Some(false), 0, 16e6);
    assert!(matches!(s.board_drive(None, 32_000, 16e6), dht::DhtAction::Answer(_)));
    assert_eq!(s.measured, Some(dht::frame(22, 40.0, 20.0)));
    s.answer_id = 7; // as its host numbers the answer
    s.set_ambient(80.0, 30.0);
    assert_eq!(s.measured, Some(dht::frame(22, 40.0, 20.0)), "the reading in flight keeps its measurement");
    assert_eq!(s.edge_due(7, dht::ANSWER_EDGES - 1, false, 16e6), Some(false));
    assert!(!s.answering);
    assert_eq!(s.measured, Some(dht::frame(22, 80.0, 30.0)));
}

// ---- engine gate cases -----------------------------------------------------

#[test]
fn a_dht11_rounds_to_tenths_before_splitting_so_the_tenth_carries() {
    assert_eq!(dht::frame(11, 42.0, 23.96), [42, 0, 24, 0, 66]);
    assert_eq!(dht::frame(11, 42.0, 49.97), [42, 0, 50, 0, 92]);
}

/// A reading at board cycle `at` (a 1.1 ms or 20 ms start), its answer run
/// to the end: the frame it sent.
fn read_at(s: &mut DhtSensor, at: u64) -> [u8; 5] {
    let start = if s.variant == 11 { 20_000 * 16 } else { 1100 * 16 };
    s.board_drive(Some(false), at, 16e6);
    let dht::DhtAction::Answer(edges) = s.board_drive(None, at + start, 16e6) else { panic!("no answer") };
    assert_eq!(edges.len(), dht::ANSWER_EDGES);
    let sent = s.measured.expect("a measurement to send");
    s.answer_id = 9;
    assert_eq!(s.edge_due(9, dht::ANSWER_EDGES - 1, false, 16e6), Some(false));
    sent
}

#[test]
fn a_dht_read_sooner_than_its_interval_sends_the_last_measurement_and_does_not_measure() {
    let sec = |x: f64| (x * 16e6) as u64;
    let mut s = DhtSensor::new(22);
    s.powered = true;
    s.powered_for = 2.0;
    s.set_ambient(40.0, 20.0);
    assert_eq!(read_at(&mut s, 0)[..2], [1, 144]);
    s.set_ambient(80.0, 20.0);
    // 0.5 s and 1 s later: the 2 s interval has not passed, so no new measurement.
    assert_eq!(read_at(&mut s, sec(0.5))[..2], [1, 144]);
    assert_eq!(read_at(&mut s, sec(1.0))[..2], [1, 144]);
    // 2.1 s after the first: it sends the old one and measures after it.
    assert_eq!(read_at(&mut s, sec(2.1))[..2], [1, 144]);
    assert_eq!(read_at(&mut s, sec(2.2))[..2], [3, 32]);
    // The DHT11 measures again after 1 s.
    let mut s = DhtSensor::new(11);
    s.powered = true;
    s.powered_for = 2.0;
    s.set_ambient(40.0, 20.0);
    assert_eq!(read_at(&mut s, 0)[0], 40);
    s.set_ambient(70.0, 20.0);
    assert_eq!(read_at(&mut s, sec(0.5))[0], 40);
    assert_eq!(read_at(&mut s, sec(1.2))[0], 40);
    assert_eq!(read_at(&mut s, sec(1.3))[0], 70);
}

#[test]
fn a_dht_that_loses_power_or_resets_mid_answer_lets_go_of_its_pin() {
    for cut in ["power", "reset"] {
        let mut cpu = board(ADAFRUIT_DHT22);
        let mut dht = DhtSensor::new(22);
        dht.powered = true;
        dht.powered_for = 2.0;
        dht.set_ambient(50.0, 20.0);
        cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht }]);
        cpu.set_input_bit(2, 2, true);
        let at = cpu.cycles;
        let dht = &mut cpu.p.parts.pins[0].dht;
        dht.board_drive(Some(false), at, 16e6);
        let dht::DhtAction::Answer(edges) = dht.board_drive(None, at + 32_000, 16e6) else { panic!("no answer") };
        let id = cpu.p.parts.begin_answer(0);
        for (i, (cycles, _)) in edges.iter().enumerate() {
            cpu.add_clock_event_arg(super::parts::EV_PIN_TARGET, super::parts::answer_edge_arg(i, 2, id), *cycles);
        }
        // Into the 80 us low that opens the answer.
        let end = cpu.cycles + 16 * 60;
        while cpu.cycles < end {
            step(&mut cpu);
        }
        assert_eq!(cpu.p.parts.target_low, 1 << 2, "{cut}: the answer holds the pin low");
        if cut == "power" {
            cpu.set_dht_powered(0, false);
        } else {
            cpu.reset_dht(0);
        }
        assert_eq!(cpu.p.parts.target_low, 0, "{cut}");
        assert_eq!(cpu.p.ports[2].pin_value & 0x04, 0x04, "{cut}: back to the pull-up");
        let end = cpu.cycles + 16 * 6000;
        while cpu.cycles < end {
            step(&mut cpu);
            assert_eq!(cpu.p.parts.target_low, 0, "{cut}: no later edge pulls");
        }
    }
}

#[test]
fn bmp280_carries_on_with_a_forced_conversion_asked_for_again_mid_way() {
    let mut b = bmp();
    b.set_ambient(21.5, 98765.4, 1.0);
    write_regs(&mut b, 1.0, &[0xf4, 0b001_001_01]);
    write_regs(&mut b, 1.003, &[0xf4, 0b101_101_01]);
    assert_eq!(read_regs(&mut b, 1.003, 0xf4, 1), vec![0b001_001_01]);
    assert_eq!(read_regs(&mut b, 1.0056, 0xf3, 1)[0] & 0x08, 0, "done at 5.5 ms, not restarted");
    assert_eq!(b.mode(), 0);
    assert_ne!(b.adc_t, 0x80000);
}

#[test]
fn ssd1306_lights_with_the_pump_off_only_on_an_externally_supplied_panel() {
    let mut external = oled();
    external.external_vcc = true;
    let mut pumped = oled();
    for d in [&mut external, &mut pumped] {
        d.start();
        for b in [0x00, 0x8d, 0x10, 0xaf] {
            d.write(b);
        }
        d.stop();
    }
    assert!(external.lit());
    assert!(!pumped.lit());
}

#[test]
fn a_dht_times_its_interval_from_each_readings_release() {
    // Released exactly 2 s apart (read_at releases 1.1 ms after `at`): the
    // second reading measures after it, whatever the replies' lengths.
    let sec = |x: f64| (x * 16e6) as u64;
    let mut s = DhtSensor::new(22);
    s.powered = true;
    s.powered_for = 2.0;
    s.set_ambient(99.9, 79.9);
    s.board_drive(Some(false), 0, 16e6);
    let dht::DhtAction::Answer(_) = s.board_drive(None, 1100 * 16, 16e6) else { panic!("no answer") };
    s.set_ambient(0.0, 0.0);
    s.answer_id = 9;
    s.edge_due(9, dht::ANSWER_EDGES - 1, false, 16e6);
    assert_eq!(s.measured_at, Some(1100 * 16));
    assert_eq!(read_at(&mut s, sec(2.0))[..2], [0, 0]);
    s.set_ambient(50.0, 20.0);
    assert_eq!(s.measured_at, Some(sec(2.0) + 1100 * 16), "measured: 2 s from release to release");
    assert_eq!(read_at(&mut s, sec(2.01))[..2], [0, 0]);
    assert_eq!(read_at(&mut s, sec(4.1))[..2], [0, 0]);
    assert_eq!(read_at(&mut s, sec(4.2))[..2], [1, 244]);
}

#[test]
fn a_dht_measures_again_after_its_board_resets() {
    let sec = |x: f64| (x * 16e6) as u64;
    // The rebuilt board's clock reads 3.5 s (0.5 s after the last
    // measurement on the old one) or 1 s (before it).
    for (told, at, measures) in [(true, 3.5, true), (false, 3.5, false), (false, 1.0, true)] {
        let mut cpu = board(ADAFRUIT_DHT22);
        let mut dht = DhtSensor::new(22);
        dht.powered = true;
        dht.powered_for = 2.0;
        dht.set_ambient(40.0, 20.0);
        cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht }]);
        assert_eq!(read_at(&mut cpu.p.parts.pins[0].dht, sec(3.0))[..2], [1, 144]);
        cpu.p.parts.pins[0].dht.set_ambient(80.0, 20.0);
        if told {
            cpu.parts_after_board_reset();
        }
        let dht = &mut cpu.p.parts.pins[0].dht;
        assert_eq!(read_at(dht, sec(at))[..2], [1, 144]);
        let next = if measures { [3, 32] } else { [1, 144] };
        assert_eq!(read_at(dht, sec(at + 0.01))[..2], next, "told {told}, {at} s");
    }
}

#[test]
fn a_dht_reset_or_unpowered_lets_go_only_of_a_hold_its_own_answer_made() {
    // A hold left on the pin by a part since replaced is not the new part's
    // to release (the reference's new part holds no line).
    let mut cpu = board(ADAFRUIT_DHT22);
    let mut dht = DhtSensor::new(22);
    dht.powered = true;
    cpu.set_pin_targets(vec![PinSlot { port: 2, bit: 2, dht }]);
    cpu.set_input_bit(2, 2, true);
    cpu.p.parts.target_low = 1 << 2;
    cpu.reset_dht(0);
    cpu.set_dht_powered(0, false);
    assert_eq!(cpu.p.parts.target_low, 1 << 2);
}
