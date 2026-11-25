//! P0-IC: the state of the five W1h logic chips (74hc595, 74hc165, cd4017, cd4060, cd4511) and the 74ls153, on one
//! clocked board, through a snapshot restore and a carried reload. Each test must go red when a chip's state is left out
//! of the snapshot or of its carry layout (proved by mutation, ship-logs/p0-ic/NOTES.md).

use crate::carry;
use crate::engine::Engine;
use crate::kinds::{self, ic_param};
use crate::program::{Device, Options, PinRecord, Program};
use crate::snapshot::SnapshotTable;

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn params(pairs: usize) -> Vec<f64> {
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 4.5;
    p[ic_param::VCC_MAX] = 5.5;
    p[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[ic_param::VIL_FIXED] = 1.0;
    p[ic_param::VIL] = 0.8;
    p[ic_param::VIH_FIXED] = 1.0;
    p[ic_param::VIH] = 2.0;
    p[ic_param::NOMINAL] = 5.0;
    p.extend(std::iter::repeat(20e-9).take(2 * pairs));
    p
}

fn ic(kind: u32, ci: usize, logic: &[usize], p: Vec<f64>, slot_base: usize) -> Device {
    let pins = [pin(1), pin(0)].into_iter().chain(logic.iter().map(|&n| pin(n))).collect();
    let mut blob = vec![slot_base as f64];
    blob.extend(std::iter::repeat(0.0).take(logic.len()));
    Device { kind, component_index: ci, pins, params: p, blob }
}

/// Device indices on the board, in program order.
pub const D595: usize = 3;
pub const D165: usize = 4;
pub const D4017: usize = 5;
pub const D4060: usize = 6;
pub const D4511: usize = 7;
pub const D153: usize = 8;

/// A 1 kHz clock on node 2 and a 300 Hz one on node 3 run a 595 (srclk 2, rclk 3, SER high, qa..qh 4..11, qh2 12), a 165
/// (cp 2, d0..d7 = 1,0,1,0,0,1,0,1, /pl high: it shifts ones in; q7 13, /q7 14), a cd4017 (clk 2, q0..q9 15..24, co 25), a
/// cd4060 (clk_in 2, q4..q14 26..35, rtc 36, ctc 37, Schmitt 2.75 / 2.25 V), a cd4511 (LE on the 300 Hz clock, BCD a on the 1 kHz one and b on the 300 Hz one, segments
/// 38..44) and a 74ls153 (select a, b from the clocks, 1y 45, 2y 46). Every output has 10 k to ground.
pub fn board() -> Program {
    let clock = |ci, node, f: f64| Device { kind: kinds::CLOCK, component_index: ci, pins: vec![pin(node), pin(0)], params: vec![f, 0.5, 5.0, 0.0, 0.0], blob: vec![0.0] };
    let mut cd4060 = params(3);
    cd4060[ic_param::SCHMITT] = 1.0;
    cd4060[ic_param::VT_PLUS] = 2.75;
    cd4060[ic_param::VT_MINUS] = 2.25;
    let mut devices = vec![
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
        clock(1, 2, 1000.0),
        clock(2, 3, 300.0),
        ic(kinds::IC_74HC595, 3, &[1, 2, 3, 0, 1, 4, 5, 6, 7, 8, 9, 10, 11, 12], params(5), 0),
        ic(kinds::IC_74HC165, 4, &[1, 2, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 13, 14], params(3), 9),
        ic(kinds::IC_CD4017, 5, &[2, 0, 0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25], params(3), 11),
        ic(kinds::IC_CD4060, 6, &[2, 0, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37], cd4060, 23),
        ic(kinds::IC_CD4511, 7, &[1, 1, 3, 2, 3, 0, 0, 38, 39, 40, 41, 42, 43, 44], params(3), 36),
        ic(kinds::IC_74LS153, 8, &[2, 3, 0, 0, 1, 0, 1, 0, 45, 0, 1, 0, 1, 46], params(1), 43),
    ];
    for (k, n) in (4..=46).enumerate() {
        devices.push(Device { kind: kinds::RESISTOR, component_index: 9 + k, pins: vec![pin(n), pin(0)], params: vec![1e4], blob: vec![] });
    }
    Program { carry: Vec::new(), node_count: 47, options: Options::default(), digital_slot_count: 45, channel_slot_count: 0, devices }
}

fn slots(e: &Engine) -> Vec<u64> {
    e.digital_slots().iter().map(|v| v.to_bits()).collect()
}

fn ic_records(buf: &[f64]) -> Vec<(usize, Vec<u64>)> {
    carry::records(buf).unwrap().into_iter().filter(|r| r.family == carry::FAMILY_IC).map(|r| (r.device, r.values.iter().map(|v| v.to_bits()).collect())).collect()
}

/// The family values of IC `device` in a carry buffer (after kind and tag).
fn family_of(buf: &[f64], device: usize) -> Vec<f64> {
    let r = carry::records(buf).unwrap().into_iter().find(|r| r.family == carry::FAMILY_IC && r.device == device).expect("an IC record");
    let n = crate::digital::sequential::SeqState::carry_len(r.values[0] as u32);
    r.values[2..2 + n].to_vec()
}

const STOPS: [f64; 8] = [2.3e-3, 3.7e-3, 5.2e-3, 16.8e-3, 17.3e-3, 22.4e-3, 27.1e-3, 40.3e-3];

#[test]
fn the_chips_state_is_nontrivial_where_the_tests_below_take_it() {
    let mut e = Engine::load(&board()).unwrap();
    e.advance(16.8e-3).unwrap();
    let buf = e.carry_take().unwrap().to_vec();
    assert_ne!(family_of(&buf, D595)[..2], [0.0, 0.0], "the 595 has shifted and latched");
    assert_ne!(family_of(&buf, D165)[0], 0.0, "the 165 has shifted ones in");
    assert_ne!(family_of(&buf, D4017)[0], 0.0, "the 4017 has counted");
    assert_ne!(family_of(&buf, D4060)[0], 0.0, "the 4060 has counted");
    assert_ne!(family_of(&buf, D4511)[0], 0.0, "the 4511 latched a nonzero code");
}

#[test]
fn a_snapshot_restore_runs_the_six_chips_on_bit_for_bit() {
    let mut e = Engine::load(&board()).unwrap();
    e.advance(STOPS[3]).unwrap();
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    let mut first = Vec::new();
    for &t in &STOPS[4..] {
        e.advance(t).unwrap();
        first.push((slots(&e), ic_records(&e.carry_take().unwrap().to_vec())));
    }
    for again in 0..2 {
        assert!(table.restore(&mut e, id));
        for (k, &t) in STOPS[4..].iter().enumerate() {
            e.advance(t).unwrap();
            let got = (slots(&e), ic_records(&e.carry_take().unwrap().to_vec()));
            assert!(got == first[k], "restore {again}: stop {t} differs");
        }
    }
}

#[test]
fn a_reload_seeded_from_a_carry_runs_the_six_chips_on_where_the_engine_it_came_from_does() {
    let p = board();
    for t in [5.2e-3, 8.9e-3, 12.4e-3, 17.1e-3] {
        let mut e = Engine::load(&p).unwrap();
        e.advance(t).unwrap();
        let buf = e.carry_take().unwrap().to_vec();
        let mut q = p.clone();
        q.options.start_time = t;
        q.carry = buf.clone();
        let mut seeded = Engine::load(&q).unwrap();
        assert_eq!(ic_records(&seeded.carry_take().unwrap().to_vec()), ic_records(&buf), "t = {t}: the IC records come back bit for bit");
        assert_eq!(slots(&seeded), slots(&e), "t = {t}: the slots at the reload");
        let mut cold_q = p.clone();
        cold_q.options.start_time = t;
        let mut cold = Engine::load(&cold_q).unwrap();
        let mut cold_differs = false;
        for dt in [3.1e-4, 1.2e-3, 2.7e-3, 6.3e-3, 11.3e-3] {
            e.advance(t + dt).unwrap();
            seeded.advance(t + dt).unwrap();
            cold.advance(t + dt).unwrap();
            assert_eq!(slots(&seeded), slots(&e), "t = {t} + {dt}: the slots");
            cold_differs |= slots(&cold) != slots(&e);
        }
        assert!(cold_differs, "t = {t}: a cold load would differ, so the carried state is what keeps the chips on course");
    }
}

#[test]
fn every_chips_carry_record_has_its_layouts_length_and_rejects_a_value_out_of_range() {
    let mut e = Engine::load(&board()).unwrap();
    e.advance(12.4e-3).unwrap();
    let buf = e.carry_take().unwrap().to_vec();
    for (dev, kind) in [(D595, kinds::IC_74HC595), (D165, kinds::IC_74HC165), (D4017, kinds::IC_CD4017), (D4060, kinds::IC_CD4060), (D4511, kinds::IC_CD4511)] {
        assert_eq!(family_of(&buf, dev).len(), crate::digital::sequential::SeqState::carry_len(kind), "device {dev}");
    }
    // A 4017 count of 12 is no count: the reader refuses the whole carry, as for a 161's.
    let mut bad = buf.clone();
    let r = carry::records(&buf).unwrap().into_iter().find(|r| r.family == carry::FAMILY_IC && r.device == D4017).unwrap();
    let at = r.values.as_ptr() as usize - buf.as_ptr() as usize;
    bad[at / std::mem::size_of::<f64>() + 2] = 12.0;
    let mut q = board();
    q.carry = bad;
    assert!(Engine::load(&q).is_err(), "a count of 12 is malformed");
}

/// A cd4060 with `clk_in` on a voltage source (`Device 1`, param 0) and `reset` grounded, every other pin on a node of its own.
fn cd4060_on_a_source(volts: f64) -> Program {
    let mut cd4060 = params(3);
    cd4060[ic_param::SCHMITT] = 1.0;
    cd4060[ic_param::VT_PLUS] = 2.75;
    cd4060[ic_param::VT_MINUS] = 2.25;
    let logic: Vec<usize> = [2, 0].into_iter().chain(3..15).collect();
    let devices = vec![
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 1, pins: vec![pin(2), pin(0)], params: vec![volts], blob: vec![] },
        ic(kinds::IC_CD4060, 2, &logic, cd4060, 0),
    ];
    Program { carry: Vec::new(), node_count: 15, options: Options::default(), digital_slot_count: 13, channel_slot_count: 0, devices }
}

#[test]
fn a_cd4060_clock_held_inside_the_hysteresis_band_keeps_its_committed_level_across_a_carried_reload() {
    // 3 V is above v_t_plus (2.75), so the Schmitt read is HIGH; then the source settles at 2.5 V, inside the band
    // (2.25 .. 2.75), which a Schmitt input holds HIGH. A reload at 2.5 V seeded without the committed level would read
    // the band as LOW against a clock remembered HIGH: a falling edge, so one count too many. Then 2 V is a real fall.
    let mut e = Engine::load(&cd4060_on_a_source(3.0)).unwrap();
    e.advance(1e-3).unwrap();
    e.set_params(&[(1, 0, 2.5)]).unwrap();
    e.advance(2e-3).unwrap();
    let count = e.digital_slots()[0];
    let buf = e.carry_take().unwrap().to_vec();
    assert_eq!(family_of(&buf, 2)[2], 1.0, "the committed Schmitt level is HIGH inside the band");
    let mut q = cd4060_on_a_source(2.5);
    q.options.start_time = 2e-3;
    q.carry = buf;
    let mut seeded = Engine::load(&q).unwrap();
    seeded.advance(3e-3).unwrap();
    e.advance(3e-3).unwrap();
    assert_eq!(seeded.digital_slots()[0], count, "held in the band: no count at the reload");
    assert_eq!(e.digital_slots()[0], count);
    e.set_params(&[(1, 0, 2.0)]).unwrap();
    seeded.set_params(&[(1, 0, 2.0)]).unwrap();
    e.advance(4e-3).unwrap();
    seeded.advance(4e-3).unwrap();
    assert_eq!(seeded.digital_slots()[0], count + 1.0, "2 V is below v_t_minus: one falling edge, counted");
    assert_eq!(e.digital_slots()[0], count + 1.0);
}

#[test]
fn a_snapshot_taken_with_a_cd4060_clock_inside_the_hysteresis_band_restores_its_committed_level() {
    // The source falls from 3 V to 2.5 V (inside 2.25 .. 2.75): the Schmitt level stays HIGH. A snapshot there, restored
    // later, must give that level back: a clock then falling to 2 V is one counted edge on both runs.
    let mut e = Engine::load(&cd4060_on_a_source(3.0)).unwrap();
    e.advance(1e-3).unwrap();
    e.set_params(&[(1, 0, 2.5)]).unwrap();
    e.advance(2e-3).unwrap();
    let count = e.digital_slots()[0];
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    e.advance(3e-3).unwrap();
    e.set_params(&[(1, 0, 2.0)]).unwrap();
    e.advance(4e-3).unwrap();
    assert_eq!(e.digital_slots()[0], count + 1.0, "the uninterrupted run counts the fall");
    assert!(table.restore(&mut e, id));
    // A move inside the band makes the chip read its input again: the held level, not a fresh LOW, decides.
    e.set_params(&[(1, 0, 2.6)]).unwrap();
    e.advance(3e-3).unwrap();
    assert_eq!(e.digital_slots()[0], count, "held in the band after the restore: no count");
    e.set_params(&[(1, 0, 2.0)]).unwrap();
    e.advance(4e-3).unwrap();
    assert_eq!(e.digital_slots()[0], count + 1.0, "the restored level is HIGH: 2 V is one falling edge");
}

// ---- P0-ICV: two cases the first pass did not run ---------------------------------------------------------------

fn load_resistors(devices: &mut Vec<Device>, first_ci: usize, nodes: std::ops::RangeInclusive<usize>) {
    for (k, n) in nodes.enumerate() {
        devices.push(Device { kind: kinds::RESISTOR, component_index: first_ci + k, pins: vec![pin(n), pin(0)], params: vec![1e4], blob: vec![] });
    }
}

/// A 595 (SRCLK 1 kHz on node 2, RCLK 300 Hz on node 3, SER high, /OE on a 40 Hz clock on node 4: high for the first
/// half period) with qa..qh on nodes 5..12 and qh2 on 13, each on 10 k to ground, so a Hi-Z pin reads 0 V.
fn board_595_oe() -> Program {
    let clock = |ci, node, f: f64| Device { kind: kinds::CLOCK, component_index: ci, pins: vec![pin(node), pin(0)], params: vec![f, 0.5, 5.0, 0.0, 0.0], blob: vec![0.0] };
    let mut devices = vec![
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
        clock(1, 2, 1000.0),
        clock(2, 3, 300.0),
        clock(3, 4, 40.0),
        ic(kinds::IC_74HC595, 4, &[1, 2, 3, 4, 1, 5, 6, 7, 8, 9, 10, 11, 12, 13], params(5), 0),
    ];
    load_resistors(&mut devices, 5, 5..=13);
    Program { carry: Vec::new(), node_count: 14, options: Options::default(), digital_slot_count: 9, channel_slot_count: 0, devices }
}

#[test]
fn a_reload_in_the_middle_of_a_595_shift_with_oe_high_keeps_both_registers_for_when_oe_falls() {
    let p = board_595_oe();
    // /OE is a 40 Hz clock: the first stop of each run is inside its high half (the pins are Hi-Z, 0 V on the pull-downs),
    // the later ones past its falling edge at 12.5 ms, where the stored byte must reach the pins.
    for t in [2.3e-3, 3.6e-3, 5.2e-3, 8.1e-3] {
        let mut e = Engine::load(&p).unwrap();
        e.advance(t).unwrap();
        assert!(e.voltages()[4] > 2.5, "t = {t}: /OE is high");
        let buf = e.carry_take().unwrap().to_vec();
        let reg = family_of(&buf, 4);
        assert!(reg[0] != 0.0 || reg[1] != 0.0, "t = {t}: the registers hold something to lose");
        assert!(e.voltages()[5..13].iter().all(|v| v.abs() < 0.05), "t = {t}: every Q is Hi-Z under /OE high");
        let mut q = p.clone();
        q.options.start_time = t;
        q.carry = buf.clone();
        let mut seeded = Engine::load(&q).unwrap();
        let mut cold_q = p.clone();
        cold_q.options.start_time = t;
        let mut cold = Engine::load(&cold_q).unwrap();
        let mut cold_differs = false;
        for dt in [3.1e-4, 4.4e-3, 9.5e-3, 12.2e-3, 17.7e-3] {
            e.advance(t + dt).unwrap();
            seeded.advance(t + dt).unwrap();
            cold.advance(t + dt).unwrap();
            assert_eq!(slots(&seeded), slots(&e), "t = {t} + {dt}: the slots");
            let bits = |x: &Engine| x.voltages()[5..14].iter().map(|v| v.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&seeded), bits(&e), "t = {t} + {dt}: the pins");
            cold_differs |= slots(&cold) != slots(&e) || bits(&cold) != bits(&e);
        }
        assert!(cold_differs, "t = {t}: a cold load would differ, so the carried registers are what the pins show once /OE falls");
    }
}

/// A 165 with /PL on a 300 Hz clock (node 3: a load every period while it is low), CP a 1 kHz clock (node 2), parallel
/// inputs 1,0,1,0,0,1,0,1, SER high, q7 on node 4 and /q7 on 5, 10 k to ground each.
fn board_165_pl() -> Program {
    let clock = |ci, node, f: f64| Device { kind: kinds::CLOCK, component_index: ci, pins: vec![pin(node), pin(0)], params: vec![f, 0.5, 5.0, 0.0, 0.0], blob: vec![0.0] };
    let mut devices = vec![
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
        clock(1, 2, 1000.0),
        clock(2, 3, 300.0),
        ic(kinds::IC_74HC165, 3, &[3, 2, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 4, 5], params(3), 0),
    ];
    load_resistors(&mut devices, 4, 4..=5);
    Program { carry: Vec::new(), node_count: 6, options: Options::default(), digital_slot_count: 2, channel_slot_count: 0, devices }
}

#[test]
fn a_snapshot_taken_between_a_165s_load_and_its_first_shift_restores_the_loaded_byte() {
    let mut e = Engine::load(&board_165_pl()).unwrap();
    // The loaded byte 0b1010_0101 (d0 is bit 0): find the instant after /PL has risen and before the next CP rise.
    let loaded = f64::from(0b1010_0101u8);
    let mut t = 0.0;
    let mut found = None;
    while t < 30e-3 {
        t += 20e-6;
        e.advance(t).unwrap();
        let buf = e.carry_take().unwrap().to_vec();
        let reg = family_of(&buf, 3);
        if e.voltages()[3] > 2.5 && reg[0] == loaded && reg[1] == 0.0 {
            found = Some(t);
            break;
        }
    }
    let t0 = found.expect("a stop with /PL released, the byte loaded and CP low");
    let mut table = SnapshotTable::default();
    let id = table.take(&e);
    let mut first = Vec::new();
    let stops = [t0 + 0.4e-3, t0 + 1.1e-3, t0 + 2.6e-3, t0 + 5.3e-3];
    for &s in &stops {
        e.advance(s).unwrap();
        first.push((slots(&e), ic_records(&e.carry_take().unwrap().to_vec())));
    }
    assert!(first.iter().any(|(_, r)| r.iter().any(|(_, v)| v.first().map_or(false, |&b| f64::from_bits(b) != loaded))), "the 165 shifted away from the loaded byte, so a restore that kept the later state would show");
    for again in 0..2 {
        assert!(table.restore(&mut e, id));
        for (k, &s) in stops.iter().enumerate() {
            e.advance(s).unwrap();
            let got = (slots(&e), ic_records(&e.carry_take().unwrap().to_vec()));
            assert!(got == first[k], "restore {again}: stop {s} differs");
        }
    }
}
