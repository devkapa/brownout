//! PSU-3 (SHIPV-C finding 1): the cold generator's boards whose one
//! source is a bench supply never decline as back-driven.
//!
//! The generator is the F5e cold fuzz's (`f5e`, `gen.rs`, hub 2: a bench
//! supply on node 1), copied unchanged so a seed here is the sweep's board.
//! A board qualifies when every device but the supply is a resistor, a
//! capacitor, an LED, a switch, a fuse or a PTC fuse (template 4 always
//! has a waveform source and template 6 is a 555: neither ever does). Run as the sweep runs it (its rated resistors, the
//! host stops of `f5e one`) to 1.2 s, its switches held where the
//! generator set them.
//!
//! Why none may decline: with its switches held, such a board's topology
//! changes only where a part opens (a fuse, a PTC, a rated resistor
//! burning), which removes a branch and moves no node at once. Every
//! capacitor is charged from the supply alone, so every node stays within
//! the ground and the setpoint (an RC network with one source has no
//! interior extremum, and an LED is a monotone two-terminal), and nothing
//! can drive current into the rail, its highest node. Toggling a switch is
//! left out because it can: a switch that closes between two charged
//! capacitors moves charge between them, and through a capacitor on the
//! rail into the supply (cold 303 tpl 3: 114 nF at 2.01 V switched onto a
//! node behind 163 uF with esr 1.15 ohm to the rail, the supply sinking
//! 0.47 A for about 0.1 us; classic lets go there too).
#![allow(dead_code, unused_assignments)]
use crate::kinds;
use crate::program::{Device, Options, PinRecord, Program};
fn pin(n: usize) -> PinRecord { PinRecord { node: n, flags: 0, open_hash: 0, dead_hash: 0 } }
fn dev(kind: u32, ci: usize, pins: &[usize], params: Vec<f64>, blob: Vec<f64>) -> Device { Device { kind, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params, blob } }

struct Rng(pub u64);
impl Rng {
    fn next(&mut self) -> u64 { self.0 ^= self.0 << 13; self.0 ^= self.0 >> 7; self.0 ^= self.0 << 17; self.0 }
    fn f(&mut self) -> f64 { (self.next() >> 11) as f64 / (1u64 << 53) as f64 }
    fn range(&mut self, lo: f64, hi: f64) -> f64 { lo + (hi - lo) * self.f() }
    fn log(&mut self, lo: f64, hi: f64) -> f64 { (lo.ln() + (hi.ln() - lo.ln()) * self.f()).exp() }
    fn pick(&mut self, n: usize) -> usize { (self.next() % n as u64) as usize }
    fn chance(&mut self, p: f64) -> bool { self.f() < p }
}

fn ls_params(delay: f64) -> Vec<f64> {
    use kinds::ic_param;
    let mut p = vec![0.0; ic_param::COUNT];
    p[ic_param::VCC_MIN] = 4.75; p[ic_param::VCC_MAX] = 5.25; p[ic_param::FAM_FLAGS] = kinds::FAM_TTL as f64;
    p[ic_param::VIL_FIXED] = 1.0; p[ic_param::VIL] = 0.8; p[ic_param::VIH_FIXED] = 1.0; p[ic_param::VIH] = 2.0;
    p[ic_param::NOMINAL] = 5.0; p[ic_param::IO_MAX] = 0.008; p[ic_param::DELAY] = delay;
    p
}

#[derive(Clone, Copy, Debug)]
enum Rated { Res { idx: usize, r: f64 }, Led { idx: usize, ifmax: f64 }, Fuse { idx: usize, r: f64, irat: f64 } }

struct Circ { pub devs: Vec<Device>, pub nc: usize, pub toggles: Vec<(f64, usize)>, pub desc: String, pub rated: Vec<Rated>, pub exact: bool, pub stepped: bool, pub trap: bool }

/// template: 0 mixed, 1 chains, 2 ICs, 3 rail-heavy, 4 waveform-heavy, 5 exotic caps
fn gen(seed: u64, hub: u32, tpl: u32) -> Circ {
    let mut g = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) ^ ((tpl as u64) << 40) | 1);
    for _ in 0..4 { g.next(); }
    let nc = 5 + g.pick(4);
    let e = if tpl == 2 || g.chance(0.3) { g.range(4.6, 5.4) } else { g.range(2.0, 14.0) };
    let mut desc = format!("tpl={tpl} hub={hub} e={e:.3}");
    let mut devs: Vec<Device> = vec![];
    let mut rated: Vec<Rated> = vec![];
    let mut ci = 0usize;
    let mut toggles = vec![];
    // node 1: the hub or an ideal source
    let src = match hub {
        0 => dev(kinds::VOLTAGE_SOURCE, 0, &[1, 0], vec![e], vec![]),
        1 => {
            let mut params = vec![0.0; kinds::battery_param::COUNT];
            params[kinds::battery_param::OCV_SCALE] = 1.0; params[kinds::battery_param::R_BASE] = g.log(0.05, 5.0);
            params[kinds::battery_param::TEMP_FACTOR] = 1.0; params[kinds::battery_param::DEP_P] = 2.0; params[kinds::battery_param::DEP_K] = 0.05;
            params[kinds::battery_param::CAPACITY_AH] = g.log(1e-4, 1.0); params[kinds::battery_param::SOC0] = g.range(0.3, 1.0);
            let lo = e * g.range(0.5, 1.0);
            desc += &format!(" bat r={:.3} lo={lo:.2}", params[kinds::battery_param::R_BASE]);
            dev(kinds::BATTERY_PACK, 0, &[1, 0], params, vec![2.0, 0.0, lo, 1.0, e])
        }
        _ => { let il = g.log(0.02, 2.0); desc += &format!(" psu ilim={il:.3}"); dev(kinds::BENCH_PSU, 0, &[1, 0], vec![e, il], vec![]) }
    };
    devs.push(src); ci += 1;
    let mut slot = 8.0;
    // extra sources
    if tpl != 6 && g.chance(0.45) {
        let v = g.range(-8.0, 12.0);
        if g.chance(0.5) { devs.push(dev(kinds::VOLTAGE_SOURCE, ci, &[2, 0], vec![v], vec![])); } else { devs.push(dev(kinds::VOLTAGE_SOURCE, ci, &[0, 2], vec![v], vec![])); }
        desc += &format!(" V2={v:.2}"); ci += 1;
    }
    if tpl != 6 && (tpl == 4 || g.chance(0.25)) {
        let n = 3;
        if g.chance(0.5) {
            let hi = g.range(1.0, 12.0); let lo = if g.chance(0.5) { 0.0 } else { g.range(-4.0, 1.0) };
            devs.push(dev(kinds::CLOCK, ci, &[n, 0], vec![g.log(2.0, 300.0), g.range(0.1, 0.9), hi, lo, 0.0], vec![slot])); slot += 1.0;
            desc += &format!(" CLK3 {lo:.1}..{hi:.1}");
        } else {
            let v1 = g.range(-3.0, 3.0); let v2 = g.range(2.0, 12.0);
            let tr = if g.chance(0.4) { g.log(1e-4, 5e-3) } else { 0.0 };
            devs.push(dev(kinds::PULSE_SOURCE, ci, &[n, 0], vec![v1, v2, g.range(0.0, 0.05), tr, tr, g.log(1e-3, 0.05), g.log(0.06, 0.4)], vec![slot])); slot += 1.0;
            desc += &format!(" PULSE3 {v1:.1}..{v2:.1} tr={tr:.0e}");
        }
        ci += 1;
        if g.chance(0.25) { let v = g.range(1.0, 5.0); devs.push(dev(kinds::VOLTAGE_SOURCE, ci, &[4, 3], vec![v], vec![])); ci += 1; desc += &format!(" stacked4-3 {v:.1}"); }
    }
    if tpl == 6 {
        // a textbook astable 555 on node 1: DIS 2, THR/TRIG 3, OUT 4, LED node 5, CV 6; ratings on every resistor
        let r1 = g.log(100.0, 2e4); let r2 = g.log(500.0, 1e5); let cap = g.log(1e-7, 2e-5);
        let mut p = vec![0.0; kinds::ic_param::COUNT];
        p[kinds::ic_param::VCC_MIN] = 2.0; p[kinds::ic_param::VCC_MAX] = 18.0; p[kinds::ic_param::NOMINAL] = 5.0;
        let mut blob = vec![slot]; blob.extend(std::iter::repeat(0.0).take(6));
        devs.push(Device { kind: kinds::NE555, component_index: ci, pins: [1usize, 0, 3, 4, 1, 6, 3, 2].iter().map(|&n| pin(n)).collect(), params: p, blob }); ci += 1;
        let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[1, 2], vec![r1], vec![])); rated.push(Rated::Res { idx, r: r1 }); ci += 1;
        let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[2, 3], vec![r2], vec![])); rated.push(Rated::Res { idx, r: r2 }); ci += 1;
        devs.push(dev(kinds::CAPACITOR, ci, &[3, 0], vec![cap, 0.0, 0.0], vec![])); ci += 1;
        devs.push(dev(kinds::CAPACITOR, ci, &[6, 0], vec![1e-8, 0.0, 0.0], vec![])); ci += 1;
        let rl = g.log(20.0, 2000.0);
        let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[4, 5], vec![rl], vec![])); rated.push(Rated::Res { idx, r: rl }); ci += 1;
        let ifmax = g.range(0.004, 0.04);
        let idx = devs.len(); devs.push(dev(kinds::LED, ci, &[5, 0], vec![g.range(1.6, 3.4), g.range(1.5, 2.5), ifmax, 0.0, 0.0, 0.0], vec![])); rated.push(Rated::Led { idx, ifmax }); ci += 1;
        if g.chance(0.5) { let r3 = g.log(10.0, 5e3); let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[4, 0], vec![r3], vec![])); rated.push(Rated::Res { idx, r: r3 }); ci += 1; }
        desc += &format!(" 555-astable R1={r1:.0} R2={r2:.0} C={cap:.1e} Rl={rl:.0}");
        let mut devs2 = devs; 
        return Circ { devs: std::mem::take(&mut devs2), nc: 7.max(nc), toggles, desc, rated, exact: g.chance(0.4), stepped: false, trap: g.chance(0.3) };
    }
    let is_fixed = |devs: &Vec<Device>, n: usize| -> bool { n == 0 || devs.iter().any(|d| matches!(d.kind, kinds::VOLTAGE_SOURCE | kinds::CLOCK | kinds::PULSE_SOURCE | kinds::BATTERY_PACK | kinds::BENCH_PSU) && d.pins.iter().any(|p| p.node == n)) };
    // a leak to ground from each node that is not fixed
    for n in 1..nc {
        if !is_fixed(&devs, n) {
            devs.push(dev(kinds::RESISTOR, ci, &[n, 0], vec![g.log(1e3, 1e6)], vec![])); ci += 1;
        }
    }
    let count = 3 + g.pick(10);
    let mut have_ic = false;
    for _ in 0..count {
        let a = g.pick(nc);
        let mut b = g.pick(nc);
        if b == a { b = (a + 1) % nc; }
        let w = g.pick(100);
        let (wr, wc, wl, ws, wf, wi) = match tpl { 1 => (60, 10, 14, 6, 6, 4), 2 => (25, 12, 18, 6, 4, 35), 3 => (30, 22, 14, 14, 6, 14), 4 => (30, 20, 20, 8, 6, 16), 5 => (20, 40, 10, 6, 4, 20), _ => (32, 14, 16, 10, 8, 20) };
        let mut acc = 0;
        let kind_sel = { acc += wr; if w < acc { 0 } else { acc += wc; if w < acc { 1 } else { acc += wl; if w < acc { 2 } else { acc += ws; if w < acc { 3 } else { acc += wf; if w < acc { 4 } else { 5 } } } } } };
        let _ = wi;
        match kind_sel {
            0 => {
                let r = g.log(1.0, 2e5);
                let idx = devs.len();
                devs.push(dev(kinds::RESISTOR, ci, &[a, b], vec![r], vec![]));
                rated.push(Rated::Res { idx, r });
                desc += &format!(" R{a}-{b}={r:.1}");
                if tpl == 1 && g.chance(0.5) { // extend a chain
                    let c2 = g.pick(nc);
                    if c2 != b { ci += 1; let r2 = g.log(1.0, 2e5); let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[b, c2], vec![r2], vec![])); rated.push(Rated::Res { idx, r: r2 }); desc += &format!(" R{b}-{c2}={r2:.1}"); }
                }
            }
            1 => {
                let c = g.log(1e-7, 2e-2);
                let (u, w2) = if tpl != 5 && g.chance(0.7) { (a, 0) } else { (a, b) };
                devs.push(dev(kinds::CAPACITOR, ci, &[u, w2], vec![c, if g.chance(0.3) { g.log(0.01, 5.0) } else { 0.0 }, 0.0], vec![]));
                desc += &format!(" C{u}-{w2}={c:.1e}");
            }
            2 => {
                let ifmax = g.range(0.004, 0.04);
                let idx = devs.len();
                devs.push(dev(kinds::LED, ci, &[a, b], vec![g.range(1.6, 3.4), g.range(1.5, 2.5), ifmax, 0.0, 0.0, 0.0], vec![]));
                rated.push(Rated::Led { idx, ifmax });
                desc += &format!(" LED{a}-{b}");
                if g.chance(0.75) { ci += 1; let r = g.log(10.0, 3000.0); let c2 = if is_fixed(&devs, a) { a } else { 1 }; let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[c2, a], vec![r], vec![])); rated.push(Rated::Res { idx, r }); desc += &format!("+R{c2}-{a}={r:.0}"); }
            }
            3 => {
                let closed = if g.chance(0.5) { 1.0 } else { 0.0 };
                let idx = devs.len();
                devs.push(dev(kinds::SWITCH, ci, &[a, b], vec![closed], vec![]));
                for _ in 0..(1 + g.pick(3)) { toggles.push((g.range(0.01, 1.2), idx)); }
                ci += 1; let r = g.log(5.0, 4000.0); let idx = devs.len(); devs.push(dev(kinds::RESISTOR, ci, &[b, 0], vec![r], vec![])); rated.push(Rated::Res { idx, r });
                desc += &format!(" SW{a}-{b}({closed})+R{b}-0={r:.0}");
            }
            4 => {
                let r = g.log(0.05, 5.0); let ir = g.log(0.01, 1.0);
                let idx = devs.len();
                if g.chance(0.7) { devs.push(dev(kinds::FUSE, ci, &[a, b], vec![r, ir], vec![])); rated.push(Rated::Fuse { idx, r, irat: ir }); desc += &format!(" F{a}-{b}={r:.2}/{ir:.2}"); }
                else { devs.push(dev(kinds::PTC_FUSE, ci, &[a, b], vec![r, ir.max(0.01)], vec![])); desc += &format!(" P{a}-{b}"); }
            }
            _ => {
                if !have_ic {
                    have_ic = true;
                    let vcc = if g.chance(0.8) { 1 } else { g.pick(nc) };
                    let gnd = if g.chance(0.9) { 0 } else { g.pick(nc) };
                    if vcc == gnd { continue; }
                    if g.chance(0.4) {
                        // 555 astable-ish: random wiring of 6 logic pins on random nodes
                        let nodes: Vec<usize> = (0..6).map(|_| g.pick(nc)).collect();
                        let mut p = vec![0.0; kinds::ic_param::COUNT];
                        p[kinds::ic_param::VCC_MIN] = 2.0; p[kinds::ic_param::VCC_MAX] = 18.0; p[kinds::ic_param::NOMINAL] = 5.0;
                        let mut pins = vec![vcc, gnd]; pins.extend(nodes.iter());
                        let mut blob = vec![slot]; blob.extend(std::iter::repeat(0.0).take(6)); slot += 1.0;
                        devs.push(Device { kind: kinds::NE555, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params: p, blob });
                        desc += &format!(" 555 vcc{vcc} gnd{gnd} {nodes:?}");
                        // timing network
                        let (thr, dis, out) = (nodes[4], nodes[5], nodes[1]);
                        ci += 1; let r1 = g.log(100.0, 1e5); devs.push(dev(kinds::RESISTOR, ci, &[vcc, dis], vec![r1], vec![])); rated.push(Rated::Res { idx: devs.len() - 1, r: r1 });
                        ci += 1; let r2 = g.log(100.0, 1e5); devs.push(dev(kinds::RESISTOR, ci, &[dis, thr], vec![r2], vec![])); rated.push(Rated::Res { idx: devs.len() - 1, r: r2 });
                        ci += 1; devs.push(dev(kinds::CAPACITOR, ci, &[thr, 0], vec![g.log(1e-8, 1e-5), 0.0, 0.0], vec![]));
                        ci += 1; let r3 = g.log(50.0, 3000.0); devs.push(dev(kinds::RESISTOR, ci, &[out, g.pick(nc)], vec![r3], vec![])); rated.push(Rated::Res { idx: devs.len() - 1, r: r3 });
                    } else {
                        let o1 = g.pick(nc); let o2 = g.pick(nc);
                        let i1 = g.pick(nc); let i2 = g.pick(nc);
                        let logic = [i1, i2, o1, 0, 0, o2, 0, 0, o2, 0, 0, o1];
                        let mut pins = vec![vcc, gnd]; pins.extend(logic.iter());
                        let mut blob = vec![0.0]; blob.extend(std::iter::repeat(0.0).take(12));
                        devs.push(Device { kind: kinds::IC_74LS00, component_index: ci, pins: pins.iter().map(|&n| pin(n)).collect(), params: ls_params(g.log(1e-8, 1e-6)), blob });
                        desc += &format!(" 74LS00 vcc{vcc} gnd{gnd} in{i1},{i2} out{o1},{o2}");
                        ci += 1; let r = g.log(40.0, 3000.0); devs.push(dev(kinds::RESISTOR, ci, &[o1, g.pick(nc)], vec![r], vec![])); rated.push(Rated::Res { idx: devs.len() - 1, r });
                        ci += 1; let r = g.log(40.0, 3000.0); devs.push(dev(kinds::RESISTOR, ci, &[o2, g.pick(nc)], vec![r], vec![])); rated.push(Rated::Res { idx: devs.len() - 1, r });
                    }
                } else {
                    let r = g.log(100.0, 5e4); devs.push(dev(kinds::RESISTOR, ci, &[a, b], vec![r], vec![]));
                }
            }
        }
        ci += 1;
    }
    toggles.sort_by(|a, b| a.0.total_cmp(&b.0));
    Circ { devs, nc, toggles, desc, rated, exact: g.chance(0.3), stepped: g.chance(0.1), trap: g.chance(0.2) }
}

/// The program with ratings added: factors relative to the span.
fn rated_program(c: &Circ, span: f64, g: &mut Rng, force_exact: Option<bool>) -> (Program, Vec<(usize, f64)>) {
    let mut devs: Vec<Device> = c.devs.iter().map(|d| Device { kind: d.kind, component_index: d.component_index, pins: d.pins.clone(), params: d.params.clone(), blob: d.blob.clone() }).collect();
    let mut pm = vec![];
    for r in &c.rated {
        if let Rated::Res { idx, r } = *r {
            let f = g.log(0.002, 2.0);
            let p_max = f * span * span / r;
            devs.push(Device { kind: kinds::RATING, component_index: devs[idx].component_index, pins: vec![], params: vec![idx as f64, p_max, r], blob: vec![] });
            pm.push((idx, p_max));
        }
    }
    let mut opts = Options::default();
    opts.no_rail_shortcut = force_exact.unwrap_or(c.exact);
    opts.force_stepped = c.stepped;
    if c.trap { opts.integration = 1; }
    (Program { node_count: c.nc, options: opts, digital_slot_count: 32, channel_slot_count: 0, devices: devs, carry: Vec::new() }, pm)
}


/// The propagated span the engine's bound computes, for this generator's source layout
/// (nodes: 1 the hub or an ideal source, 2 a signed source, 3 a clock/pulse, 4 a source stacked on 3).
fn bound_span(c: &Circ) -> f64 {
    let mut rng: Vec<Option<(f64, f64)>> = vec![None; c.nc];
    rng[0] = Some((0.0, 0.0));
    let mut ds: Vec<&Device> = c.devs.iter().filter(|d| matches!(d.kind, kinds::VOLTAGE_SOURCE | kinds::CLOCK | kinds::PULSE_SOURCE | kinds::BATTERY_PACK | kinds::BENCH_PSU)).collect();
    // resolve in dependency order (a stacked source needs its parent first)
    for _ in 0..4 {
        for d in ds.iter() {
            let (a, b) = (d.pins[0].node, d.pins[1].node); // pos, neg
            let (lo, hi) = match d.kind {
                kinds::VOLTAGE_SOURCE => (d.params[0], d.params[0]),
                kinds::BENCH_PSU => { let v = d.params[0]; if v < 0.0 { (v, 0.0) } else { (0.0, v) } }
                kinds::CLOCK => (d.params[2].min(d.params[3]), d.params[2].max(d.params[3])),
                kinds::PULSE_SOURCE => (d.params[0].min(d.params[1]), d.params[0].max(d.params[1])),
                kinds::BATTERY_PACK => { let v = *d.blob.last().unwrap(); let l = d.blob[2]; (l.min(v).min(0.0), l.max(v).max(0.0)) }
                _ => continue,
            };
            if let Some((plo, phi)) = rng[b] { if rng[a].is_none() { rng[a] = Some((plo + lo, phi + hi)); } }
            else if let Some((plo, phi)) = rng[a] { if rng[b].is_none() { rng[b] = Some((plo - hi, phi - lo)); } }
        }
    }
    ds.clear();
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for r in rng.iter().flatten() { lo = lo.min(r.0); hi = hi.max(r.1); }
    (hi - lo).max(0.5)
}

fn qualifies(c: &Circ) -> bool {
    c.devs.iter().skip(1).all(|d| matches!(d.kind, kinds::RESISTOR | kinds::CAPACITOR | kinds::LED | kinds::SWITCH | kinds::FUSE | kinds::PTC_FUSE))
}

/// Runs every qualifying board of template `tpl` for seeds 1 to 300, in
/// exact mode or not, and returns how many ran.
fn sweep(tpl: u32, exact_mode: bool) -> usize {
    let mut n = 0;
    for seed in 1..=300u64 {
        let c = gen(seed, 2, tpl);
        if !qualifies(&c) {
            continue;
        }
        let span = bound_span(&c);
        let mut gr = Rng(seed ^ 0x5555 ^ (tpl as u64) << 32 ^ (2u64) << 8);
        let (prog, _) = rated_program(&c, span, &mut gr, Some(exact_mode));
        let what = format!("seed {seed} tpl {tpl} exact {exact_mode} ({})", c.desc);
        n += 1;
        let mut e = match crate::engine::Engine::load(&prog) {
            Ok(e) => e,
            Err(err) => {
                assert!(!err.message.starts_with(crate::solve::BACK_DRIVE_REASON), "{what}: declined at load: {}", err.message);
                continue;
            }
        };
        let mut g = Rng(seed ^ 0xABCDEF123);
        let mut t = 0.0f64;
        while t < 1.2 {
            let next = (t + g.log(1e-4, 0.05)).min(1.2);
            if let Err(err) = e.advance(next) {
                assert!(!err.message.starts_with(crate::solve::BACK_DRIVE_REASON), "{what}: {}", err.message);
                break;
            }
            t = next;
        }
    }
    n
}

// One test per template and mode, so the sweep runs on the test threads.
macro_rules! one_source_sweep {
    ($($name:ident: $tpl:expr, $exact:expr;)*) => {$(
        #[test]
        fn $name() {
            assert!(sweep($tpl, $exact) > 0, "no board qualified");
        }
    )*};
}

one_source_sweep! {
    a_one_source_board_of_template_0_never_declines_as_back_driven_default: 0, false;
    a_one_source_board_of_template_0_never_declines_as_back_driven_exact: 0, true;
    a_one_source_board_of_template_1_never_declines_as_back_driven_default: 1, false;
    a_one_source_board_of_template_1_never_declines_as_back_driven_exact: 1, true;
    a_one_source_board_of_template_2_never_declines_as_back_driven_default: 2, false;
    a_one_source_board_of_template_2_never_declines_as_back_driven_exact: 2, true;
    a_one_source_board_of_template_3_never_declines_as_back_driven_default: 3, false;
    a_one_source_board_of_template_3_never_declines_as_back_driven_exact: 3, true;
    a_one_source_board_of_template_5_never_declines_as_back_driven_default: 5, false;
    a_one_source_board_of_template_5_never_declines_as_back_driven_exact: 5, true;
}

#[test]
fn templates_4_and_6_never_qualify() {
    // Every template-4 board has a clock or a pulse source, and every
    // template-6 board is a 555 astable (`gen`).
    assert!((1..=300u64).all(|seed| !qualifies(&gen(seed, 2, 4)) && !qualifies(&gen(seed, 2, 6))));
}

#[test]
fn the_one_source_sweep_covers_two_hundred_boards() {
    // Seeds 1 to 300 give 217 qualifying boards over templates 0, 1, 2, 3
    // and 5, each run in both modes by the tests above.
    let n: usize = [0u32, 1, 2, 3, 5].iter().map(|&tpl| (1..=300u64).filter(|&seed| qualifies(&gen(seed, 2, tpl))).count()).sum();
    assert_eq!(n, 217);
}
