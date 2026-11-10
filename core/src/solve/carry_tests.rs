//! W2c: what `bo_engine_carry_take` reads off an engine (core/README.md,
//! "Carry"). Each family's record is held to what the engine already
//! publishes: capacitor voltages to the node voltages across an esr-free
//! capacitor, battery, PSU, PTC and thermal values to `bo_engine_device_state`,
//! a latched failure to its record, an IC's drivers to the kernel's table.

use super::rail_tests::{cell_switch_into_an_led_loop, cold_2575_tpl2, cold_757_tpl0, two_ptc_branches, STOPS_2575};
use super::slow_tests::{ring_behind_a_fuse, rollback_rig};
use crate::carry::{self, Record};
use crate::digital;
use crate::engine::Engine;
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};

fn take(e: &mut Engine) -> Vec<f64> {
    e.carry_take().unwrap().to_vec()
}

fn family<'a>(buf: &'a [f64], family: u32) -> Vec<Record<'a>> {
    carry::records(buf).unwrap().into_iter().filter(|r| r.family == family).collect()
}

fn one<'a>(buf: &'a [f64], fam: u32, device: usize) -> Record<'a> {
    let mut hits = family(buf, fam).into_iter().filter(|r| r.device == device);
    let r = hits.next().unwrap_or_else(|| panic!("no family {fam} record for device {device}"));
    assert!(hits.next().is_none(), "one record per device and family");
    r
}

fn advanced(p: &Program, stops: &[f64]) -> Engine {
    let mut e = Engine::load(p).unwrap();
    for &t in stops {
        e.advance(t).unwrap();
    }
    e
}

#[test]
fn every_record_is_versioned_and_the_buffer_reads_back_whole() {
    let mut e = advanced(&cold_2575_tpl2(), &STOPS_2575[..20]);
    let buf = take(&mut e);
    let records = carry::records(&buf).unwrap();
    assert!(records.len() > 5, "{} records", records.len());
    assert!(records.iter().all(|r| r.version == carry::VERSION));
    assert!(carry::records(&buf[..buf.len() - 1]).is_err(), "a cut buffer is refused");
}

#[test]
fn a_carry_holds_every_capacitors_voltage_at_the_engines_time() {
    // On every partition kind: 2575 tpl 2 (a CC rail capacitor and stepped
    // and closed-form ones), the PTC board's two 1 mF capacitors, and the
    // cell board's 7 uF one. With no esr, the charge's voltage is the one
    // across the pins.
    for (name, p, stops) in [
        ("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..25].to_vec()),
        ("two PTCs", two_ptc_branches(), vec![1e-3, 2e-3]),
        ("cell loop", cell_switch_into_an_led_loop(), vec![0.01, 0.05]),
    ] {
        let mut e = advanced(&p, &stops);
        let buf = take(&mut e);
        let caps: Vec<(usize, &Device)> = p.devices.iter().enumerate().filter(|(_, d)| d.kind == kinds::CAPACITOR).collect();
        assert!(!caps.is_empty());
        assert_eq!(family(&buf, carry::FAMILY_CAPACITOR).len(), caps.len(), "{name}: one record per capacitor");
        for (i, d) in caps {
            let r = one(&buf, carry::FAMILY_CAPACITOR, i);
            assert_eq!(r.values.len(), 1);
            if d.params[1] == 0.0 {
                let across = e.voltages()[d.pins[0].node] - e.voltages()[d.pins[1].node];
                assert!((r.values[0] - across).abs() <= 1e-9 * (1.0 + across.abs()), "{name}: capacitor {i}: vc {} against {across} across it", r.values[0]);
            }
        }
    }
}

#[test]
fn a_carry_holds_the_battery_the_psu_the_ptcs_and_the_thermal_state_as_device_state_reads_them() {
    let mut e = advanced(&cell_switch_into_an_led_loop(), &[0.01, 0.05]);
    let buf = take(&mut e);
    let battery = one(&buf, carry::FAMILY_BATTERY, 0).values.to_vec();
    assert_eq!(battery, e.device_state(0).unwrap().to_vec(), "the battery as device_state reads it");

    let p = cold_2575_tpl2();
    let mut e = advanced(&p, &STOPS_2575[..25]);
    let buf = take(&mut e);
    let psu = p.devices.iter().position(|d| d.kind == kinds::BENCH_PSU).unwrap();
    assert_eq!(one(&buf, carry::FAMILY_PSU, psu).values.to_vec(), e.device_state(psu as u32).unwrap().to_vec());

    let mut e = advanced(&two_ptc_branches(), &[1e-3, 2e-3]);
    let buf = take(&mut e);
    for ptc in [1usize, 4] {
        let slow = one(&buf, carry::FAMILY_SLOW, ptc).values.to_vec();
        assert_eq!(slow.len(), carry::slow::LEN);
        assert_eq!(slow[4..7].to_vec(), e.device_state(ptc as u32).unwrap().to_vec(), "PTC {ptc}: tripped, stress, rt");
    }

    let p = rollback_rig();
    let mut e = advanced(&p, &[0.25, 0.5]);
    let buf = take(&mut e);
    let led = p.devices.iter().position(|d| d.kind == kinds::LED).unwrap();
    let state = e.device_state(led as u32).unwrap().to_vec();
    let slow = one(&buf, carry::FAMILY_SLOW, led).values.to_vec();
    assert_eq!((slow[2], slow[3]), (state[0], state[2]), "the LED's temperature and shutdown");
}

#[test]
fn a_latched_failure_carries_with_its_record() {
    // The fuse behind the 74LS04 trips at about 17 ms.
    let mut e = advanced(&ring_behind_a_fuse(), &[0.01, 0.03]);
    let latched = e.records(0).unwrap().to_vec();
    assert_eq!(latched.len(), 7, "the fuse has tripped");
    let fuse = latched[0] as usize;
    let buf = take(&mut e);
    let slow = one(&buf, carry::FAMILY_SLOW, fuse).values.to_vec();
    assert_eq!(slow[0], 1.0, "failed");
    assert_eq!(one(&buf, carry::FAMILY_LATCH, fuse).values.to_vec(), latched[1..7].to_vec(), "its latch record");
}

#[test]
fn a_carry_holds_each_ics_family_state_edge_history_and_drivers() {
    // 2575 tpl 2's 555 and the rollback rig's 74LS00: one record per IC,
    // its kind and family tag first, and one (committed, pending target,
    // pending due) triple per output driver, as the kernel's table holds it.
    for (name, p, stops) in [("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..25].to_vec()), ("rollback rig", rollback_rig(), vec![0.25, 0.5])] {
        let mut e = advanced(&p, &stops);
        let buf = take(&mut e);
        let ics: Vec<usize> = p.devices.iter().enumerate().filter(|(_, d)| digital::is_ic(d.kind)).map(|(i, _)| i).collect();
        assert!(!ics.is_empty());
        assert_eq!(family(&buf, carry::FAMILY_IC).len(), ics.len(), "{name}: one record per IC");
        for i in ics {
            let r = one(&buf, carry::FAMILY_IC, i);
            assert_eq!(r.values[0], p.devices[i].kind as f64, "{name}: device {i}'s kind");
            let drivers = e.debug_ic_drivers(i);
            let tail = &r.values[r.values.len() - 3 * drivers.len()..];
            assert_eq!(r.values[r.values.len() - 3 * drivers.len() - 1], drivers.len() as f64, "{name}: device {i}'s driver count");
            for (k, (committed, pending)) in drivers.iter().enumerate() {
                assert_eq!(tail[3 * k], carry::ic::mode(*committed), "{name}: device {i} driver {k} committed");
                match pending {
                    Some((target, due)) => assert_eq!((tail[3 * k + 1], tail[3 * k + 2]), (carry::ic::mode(Some(*target)), *due)),
                    None => assert!(tail[3 * k + 1] == -1.0 && tail[3 * k + 2].is_nan()),
                }
            }
        }
        let ne555 = p.devices.iter().position(|d| d.kind == kinds::NE555);
        if let Some(i) = ne555 {
            assert_eq!(one(&buf, carry::FAMILY_IC, i).values[1], carry::ic::NE555);
        }
    }
}

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn w1d_params(delay_pairs: usize, extra: &[f64]) -> Vec<f64> {
    let mut p = vec![0.0; kinds::ic_param::COUNT];
    p[kinds::ic_param::VCC_MIN] = 4.5;
    p[kinds::ic_param::VCC_MAX] = 5.5;
    p[kinds::ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[kinds::ic_param::VIL_FIXED] = 1.0;
    p[kinds::ic_param::VIL] = 0.8;
    p[kinds::ic_param::VIH_FIXED] = 1.0;
    p[kinds::ic_param::VIH] = 2.0;
    p[kinds::ic_param::NOMINAL] = 5.0;
    p.extend(std::iter::repeat(20e-9).take(2 * delay_pairs));
    p.extend_from_slice(extra);
    p
}

fn w1d_ic(kind: u32, ci: usize, logic_len: usize, params: Vec<f64>, rom_words: usize, slot_base: f64) -> Device {
    let pins = std::iter::once(pin(1)).chain(std::iter::once(pin(0))).chain((0..logic_len).map(|_| pin(0))).collect();
    let mut blob = vec![slot_base];
    blob.extend(std::iter::repeat(0.0).take(logic_len + rom_words));
    Device { kind, component_index: ci, pins, params, blob }
}

#[test]
fn the_sequential_and_memory_families_carry_their_registers_and_write_state() {
    // One each of 161, 173, 74hc74, 189 and 28c16 on 5 V, every logic pin
    // grounded; the 189 seeded with 0..15.
    let devices = vec![
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
        w1d_ic(kinds::IC_74LS161, 1, 14, w1d_params(4, &[]), 0, 0.0),
        w1d_ic(kinds::IC_74LS173, 2, 14, w1d_params(4, &[]), 0, 5.0),
        w1d_ic(kinds::IC_74HC74, 3, 12, w1d_params(2, &[]), 0, 9.0),
        w1d_ic(kinds::IC_74LS189, 4, 14, w1d_params(5, &[]), 0, 13.0),
        w1d_ic(kinds::IC_28C16, 5, 22, w1d_params(5, &[10e-3, 0.0]), 2048 / 4, 17.0),
    ];
    let p = Program { carry: Vec::new(), node_count: 2, options: Options::default(), digital_slot_count: 25, channel_slot_count: 0, devices };
    let mut e = Engine::load(&p).unwrap();
    let nibbles: Vec<f64> = (0..16).map(|k| k as f64).collect();
    e.seed_memory(4, &nibbles).unwrap();
    e.advance(1e-6).unwrap();
    let buf = take(&mut e);
    let tag = |i: usize| one(&buf, carry::FAMILY_IC, i).values[1];
    assert_eq!([tag(1), tag(2), tag(3), tag(4), tag(5)], [carry::ic::SEQUENTIAL, carry::ic::SEQUENTIAL, carry::ic::SEQUENTIAL, carry::ic::RAM_189, carry::ic::EEPROM]);
    let ram = one(&buf, carry::FAMILY_IC, 4).values.to_vec();
    assert_eq!(ram[2..18].to_vec(), nibbles, "the 189's sixteen nibbles");
    let eeprom = one(&buf, carry::FAMILY_IC, 5).values.to_vec();
    assert_eq!(eeprom[2], carry::ic::image_hash(&[0u8; 2048]), "the 28c16's image by hash");
    assert_eq!(eeprom[3], 2048.0);
    assert!(eeprom[6].is_nan(), "no write in flight");
}

#[test]
fn a_carry_taken_at_an_r86_yield_is_the_carry_taken_at_a_plain_stop_there() {
    // A carry is taken between calls; an advance_for that yielded leaves
    // the last completed instant with no horizon clamp, its stepped
    // partitions each at its own last step. The take brings the dynamic
    // partitions to the engine's time first, as advance(time) does, so it
    // reads what a host stopping there plainly would. Every yield of a host
    // step on 757 tpl 0 and 2575 tpl 2 is checked: bit for bit, since the
    // run is the same and the take clamps a copy (R101: an earlier take that
    // clamped the live engine changed the run after it, which read as a
    // cadence difference of up to 3.3 mV on 2575's device 9).
    let boards: Vec<(&str, Program, Vec<f64>, f64)> = vec![
        ("757 tpl 0", cold_757_tpl0(), vec![0.006272881300476146, 0.011116023500639817, 0.012362155978723055, 0.037731840680085206, 0.03788763379837714, 0.053120364878471986], 0.07025570911480136),
        ("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..2].to_vec(), STOPS_2575[2]),
    ];
    let mut differed = 0;
    for (name, p, before, t_end) in &boards {
        let mut now = 0.0;
        let mut clock = move || {
            now += 1.0;
            now
        };
        let mut e = advanced(p, before);
        let mut yields = 0;
        while !e.advance_for(*t_end, Some(crate::kernel::Budget { clock: &mut clock, budget_ms: 2.0, check_every: 1 })).unwrap() {
            yields += 1;
            let t_y = e.time();
            // The state at the yield, before the take clamps it.
            let unclamped = e.snapshot();
            let at_yield = take(&mut e);
            let mut plain = advanced(p, before);
            plain.advance(t_y).unwrap();
            let at_stop = take(&mut plain);
            let bits = |v: &[f64]| v.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&at_yield), bits(&at_stop), "{name}: yield {yields} at t = {t_y}");
            let mut raw = unclamped;
            let mut out = Vec::new();
            raw.debug_carry_unclamped(&mut out);
            if bits(&out) != bits(&at_yield) {
                differed += 1;
            }
            if yields >= 40 {
                break;
            }
        }
        assert!(yields > 0, "{name}: it yielded");
    }
    assert!(differed > 0, "at some yield the unclamped state differs, so the clamp in the take matters");
}

#[test]
fn taking_a_carry_at_every_yield_leaves_the_run_bit_identical() {
    // The take clamps a copy, never the live engine: a run that takes a
    // carry at each yield and goes on equals one that never takes one.
    for (name, p, before, t_end) in [
        ("757 tpl 0", cold_757_tpl0(), vec![0.006272881300476146, 0.011116023500639817, 0.012362155978723055, 0.037731840680085206, 0.03788763379837714, 0.053120364878471986], 0.07025570911480136),
        ("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..2].to_vec(), STOPS_2575[2]),
    ] {
        let run = |taking: bool| {
            let mut now = 0.0;
            let mut clock = move || {
                now += 1.0;
                now
            };
            let mut e = advanced(&p, &before);
            let mut rows = Vec::new();
            while !e.advance_for(t_end, Some(crate::kernel::Budget { clock: &mut clock, budget_ms: 2.0, check_every: 1 })).unwrap() {
                if taking {
                    take(&mut e);
                }
                rows.push(e.voltages().iter().chain(e.currents()).map(|v| v.to_bits()).collect::<Vec<_>>());
            }
            rows.push(e.voltages().iter().chain(e.currents()).map(|v| v.to_bits()).collect());
            (rows, (0..28).map(|i| e.stat(i).to_bits()).collect::<Vec<_>>())
        };
        let (plain, plain_stats) = run(false);
        let (taken, taken_stats) = run(true);
        assert!(plain.len() > 1, "{name}: it yielded");
        assert!(plain == taken, "{name}: taking carries changed the run");
        assert_eq!(plain_stats, taken_stats, "{name}: and its stats");
    }
}

// ---- the seed (program v4's CARRY block, W2c piece 2) -------------------

/// `p` loaded at `t` with `carry` in its block, as the host's carrying load
/// builds it: the same devices, `START_TIME` the engine's time.
fn reload(p: &Program, carry: &[f64], t: f64) -> Engine {
    let mut q = p.clone();
    q.options.start_time = t;
    q.carry = carry.to_vec();
    Engine::load(&q).unwrap()
}

fn bits_of(buf: &[f64], families: &[u32]) -> Vec<(usize, u32, Vec<u64>)> {
    carry::records(buf).unwrap().into_iter().filter(|r| families.contains(&r.family)).map(|r| (r.device, r.family, r.values.iter().map(|v| v.to_bits()).collect())).collect()
}

/// The families this piece seeds (an IC's is piece 3's).
const SEEDED: [u32; 5] = [carry::FAMILY_CAPACITOR, carry::FAMILY_BATTERY, carry::FAMILY_PSU, carry::FAMILY_SLOW, carry::FAMILY_LATCH];

#[test]
fn a_load_seeded_from_a_carry_holds_that_carry() {
    // Taken, seeded into a fresh load of the same program at the same time,
    // and taken again: the same values, bit for bit, but for a closed-form
    // capacitor's, which its first segment re-anchors at the load (`inf +
    // (vc0 - inf) e^0` can round an ulp off `vc0`). On 2575 tpl 2 (a CC
    // rail capacitor, stepped and closed-form ones, the PSU in CC), the PTC
    // board (two PTCs, a limited PSU), the cell board (the battery, LEDs),
    // the rollback rig (a rated resistor's stress, an LED's temperature and
    // held junction temperature) and the fuse ring (a latched fuse).
    for (name, p, stops) in [
        ("2575 tpl 2", cold_2575_tpl2(), STOPS_2575[..25].to_vec()),
        ("two PTCs", two_ptc_branches(), vec![1e-3, 2e-3]),
        ("cell loop", cell_switch_into_an_led_loop(), vec![0.01, 0.05]),
        ("rollback rig", rollback_rig(), vec![0.25, 0.5]),
        ("fuse ring", ring_behind_a_fuse(), vec![0.01, 0.03]),
    ] {
        let mut e = advanced(&p, &stops);
        let t = e.time();
        let buf = take(&mut e);
        if name == "2575 tpl 2" {
            // Pins the regime rule: the PSU is in CC here, and the reload,
            // which does not seed it, comes back in CC from the charges.
            let psu = p.devices.iter().position(|d| d.kind == kinds::BENCH_PSU).unwrap();
            assert_eq!(one(&buf, carry::FAMILY_PSU, psu).values, &[2.0], "{name}: the PSU is in CC");
        }
        let mut seeded = reload(&p, &buf, t);
        let again = take(&mut seeded);
        let (want, got) = (bits_of(&buf, &SEEDED), bits_of(&again, &SEEDED));
        assert!(!want.is_empty());
        for (w, g) in want.iter().zip(&got) {
            if w.1 == carry::FAMILY_CAPACITOR && (w.0, w.1) == (g.0, g.1) {
                let (a, b) = (f64::from_bits(w.2[0]), f64::from_bits(g.2[0]));
                assert!((a - b).abs() <= 4.0 * f64::EPSILON * a.abs(), "{name}: capacitor {}: {a} against {b}", w.0);
                continue;
            }
            assert_eq!(w, g, "{name}: device {} family {}: {:?} against {:?}", w.0, w.1, w.2.iter().map(|b| f64::from_bits(*b)).collect::<Vec<_>>(), g.2.iter().map(|b| f64::from_bits(*b)).collect::<Vec<_>>());
        }
        assert_eq!(want.len(), got.len(), "{name}: as many records");
    }
}

#[test]
fn a_reload_seeded_from_a_carry_runs_on_within_w0_of_the_engine_it_was_taken_from() {
    // No IC on these, so everything the run reads is seeded. The reload's
    // first step is backward Euler from the carried charges, so analog is
    // within W0 (100 uV + 0.1 %), not bitwise; the PTCs' and the cell's
    // device state likewise.
    // The PTC board's capacitors are mid-charge at the reload, so a cold
    // load is far off there; the cell board has settled by then, and holds
    // the battery's charge.
    for (name, p, stops, later, cold_differs) in [
        ("two PTCs", two_ptc_branches(), vec![1e-3, 2e-3], vec![3e-3, 5e-3, 0.01, 0.05], true),
        ("cell loop", cell_switch_into_an_led_loop(), vec![0.01, 0.05], vec![0.06, 0.1, 0.5], false),
    ] {
        let mut e = advanced(&p, &stops);
        let t = e.time();
        let buf = take(&mut e);
        let mut seeded = reload(&p, &buf, t);
        let mut cold = reload(&p, &[], t);
        let mut cold_worst = 0.0f64;
        for &t1 in &later {
            e.advance(t1).unwrap();
            seeded.advance(t1).unwrap();
            cold.advance(t1).unwrap();
            for (n, (&g, &w)) in seeded.voltages().iter().zip(e.voltages()).enumerate() {
                let w0 = (g - w).abs() / (1e-4 + 1e-3 * g.abs().max(w.abs()));
                assert!(w0 <= 1.0, "{name}: node {n} at {t1}: {g} against {w} ({w0:.2} W0)");
            }
            cold_worst = cold.voltages().iter().zip(e.voltages()).map(|(&g, &w)| (g - w).abs() / (1e-4 + 1e-3 * g.abs().max(w.abs()))).fold(cold_worst, f64::max);
        }
        assert!(!cold_differs || cold_worst > 1.0, "{name}: a cold load would not do ({cold_worst:.2} W0), so the seed is what holds it");
    }
}

#[test]
fn a_carried_fuse_stays_open_and_is_not_announced_again() {
    let p = ring_behind_a_fuse();
    let mut e = advanced(&p, &[0.01, 0.03]);
    let latched = e.records(0).unwrap().to_vec();
    assert_eq!(latched.len(), 7, "the fuse has tripped");
    let fuse = latched[0] as usize;
    let buf = take(&mut e);
    let mut seeded = reload(&p, &buf, 0.03);
    assert_eq!(seeded.records(0).unwrap().to_vec(), latched, "still latched, with its record");
    assert!(seeded.records(3).unwrap().is_empty(), "not in the latch log: the host announced it before the reload");
    assert_eq!(seeded.currents()[fuse], 0.0, "open");
    seeded.advance(0.05).unwrap();
    assert_eq!(seeded.currents()[fuse], 0.0, "and stays open");
}

#[test]
fn a_twelve_volt_charge_carried_onto_five_volts_stresses_a_resistor_cold_at_five_volts() {
    // Ruling R57 D10 (a): 5 V through 10 ohm (rated 3 W) into 2 F to ground
    // is at most 2.5 W, so the resistor is cold (never stressed) on a cold
    // load. Carried at 12 V, the capacitor puts 7 V across it: 4.9 W,
    // decaying with tau/2 = 10 s, so the stress `S(t) = (P0/pMax) 10 (1 -
    // e^(-t/10)) - t` reaches 1 at the root below (2.1192 s). The span
    // widened by the carried charge makes it hot.
    let p = {
        let mut devices = vec![
            Device { kind: kinds::VOLTAGE_SOURCE, component_index: 0, pins: vec![pin(1), pin(0)], params: vec![5.0], blob: vec![] },
            Device { kind: kinds::RESISTOR, component_index: 1, pins: vec![pin(1), pin(2)], params: vec![10.0], blob: vec![] },
            Device { kind: kinds::CAPACITOR, component_index: 2, pins: vec![pin(2), pin(0)], params: vec![2.0, 0.0, 0.0], blob: vec![] },
        ];
        devices.push(Device { kind: kinds::RATING, component_index: 1, pins: vec![], params: vec![1.0, 3.0, 10.0], blob: vec![] });
        Program { carry: Vec::new(), node_count: 3, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
    };
    let ratio = 4.9 / 3.0;
    let s = |t: f64| ratio * 10.0 * (1.0 - (-t / 10.0f64).exp()) - t;
    let (mut lo, mut hi) = (0.0, 5.0);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if s(mid) < 1.0 { lo = mid } else { hi = mid }
    }
    let hand = 0.5 * (lo + hi);
    assert!((hand - 2.1192).abs() < 1e-4, "hand value {hand}");

    let mut cold = Engine::load(&p).unwrap();
    cold.advance(4.0).unwrap();
    assert!(cold.records(0).unwrap().is_empty(), "cold load: never stressed");

    let mut carried = reload(&p, &[2.0, carry::FAMILY_CAPACITOR as f64, carry::VERSION as f64, 1.0, 12.0], 0.0);
    assert!((carried.voltages()[2] - 12.0).abs() < 1e-9, "the capacitor starts at its charge");
    carried.advance(4.0).unwrap();
    let rec = carried.records(0).unwrap().to_vec();
    assert_eq!(rec.len(), 7, "carried: the resistor latches");
    assert_eq!(rec[0], 1.0);
    assert!((rec[3] - hand).abs() <= 1e-6 * hand, "at {} against the hand value {hand}", rec[3]);
}

#[test]
fn a_carry_taken_at_an_r86_yield_seeds_the_load_a_plain_stops_carry_does() {
    // The take at a yield is the plain stop's, bit for bit (above); so are
    // the loads seeded from them, run on.
    let p = two_ptc_branches();
    let mut now = 0.0;
    let mut clock = move || {
        now += 1.0;
        now
    };
    let mut e = advanced(&p, &[1e-3]);
    let mut yields = 0;
    while !e.advance_for(0.02, Some(crate::kernel::Budget { clock: &mut clock, budget_ms: 2.0, check_every: 1 })).unwrap() {
        yields += 1;
        if yields % 7 != 1 {
            continue;
        }
        let t_y = e.time();
        let at_yield = take(&mut e);
        let mut plain = advanced(&p, &[1e-3]);
        plain.advance(t_y).unwrap();
        let at_stop = take(&mut plain);
        let mut a = reload(&p, &at_yield, t_y);
        let mut b = reload(&p, &at_stop, t_y);
        for t1 in [t_y + 1e-4, t_y + 1e-3] {
            a.advance(t1).unwrap();
            b.advance(t1).unwrap();
            let bits = |e: &Engine| e.voltages().iter().chain(e.currents()).map(|v| v.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&a), bits(&b), "yield {yields} at {t_y}, run to {t1}");
        }
        if yields > 30 {
            break;
        }
    }
    assert!(yields > 1, "it yielded");
}
