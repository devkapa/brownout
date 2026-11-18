//! A capacitor straight across a cell's rail (core/README.md, Rails point 7,
//! "A cell's rail capacitor", CAPI). The rail is algebraic and the capacitor
//! follows it, so its published current is `C dV/dt` of the rail as the
//! solver publishes it, and the cell's own current closes KCL at the rail
//! node with it. Every expected value below is derived by hand from the
//! board's own netlist in its comment; the right-hand rate of the published
//! rail is read by a snapshot advanced 1 ns past the stop and put back, so
//! the run itself is the unprobed one.

use super::*;
use crate::engine::Engine;
use crate::kinds::battery_param;
use crate::program::{Device, Options, PinRecord};

fn pin(node: usize) -> PinRecord {
    PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
}

fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
    Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
}

fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
    Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
}

fn capacitor(ci: usize, a: usize, b: usize, c: f64, esr: f64) -> Device {
    Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, esr, 0.0], blob: vec![] }
}

fn switch(ci: usize, a: usize, b: usize, closed: bool) -> Device {
    Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![if closed { 1.0 } else { 0.0 }], blob: vec![] }
}

fn led(ci: usize, anode: usize, cathode: usize) -> Device {
    // vf 2.0 V, n 2, 20 mA rated.
    Device { kind: kinds::LED, component_index: ci, pins: vec![pin(anode), pin(cathode)], params: vec![2.0, 2.0, 0.02, 0.0, 0.0, 0.0], blob: vec![] }
}

/// A cell of constant OCV `e` and constant resistance `r` (no depletion),
/// 1 Ah, full.
fn battery(ci: usize, pos: usize, neg: usize, e: f64, r: f64) -> Device {
    let mut params = vec![0.0; battery_param::COUNT];
    params[battery_param::OCV_SCALE] = 1.0;
    params[battery_param::R_BASE] = r;
    params[battery_param::TEMP_FACTOR] = 1.0;
    params[battery_param::DEP_P] = 2.0;
    params[battery_param::DEP_K] = 0.05;
    params[battery_param::CAPACITY_AH] = 1.0;
    params[battery_param::SOC0] = 1.0;
    Device { kind: kinds::BATTERY_PACK, component_index: ci, pins: vec![pin(pos), pin(neg)], params, blob: vec![1.0, 0.5, e] }
}

fn program(node_count: usize, devices: Vec<Device>) -> Program {
    Program { carry: Vec::new(), node_count, options: Options::default(), digital_slot_count: 0, channel_slot_count: 0, devices }
}

fn exact(mut p: Program) -> Program {
    p.options.no_rail_shortcut = true;
    p
}

/// W0's current tolerance, 1 nA + 1e-3 of the value.
fn w0_i(want: f64) -> f64 {
    1e-9 + 1e-3 * want.abs()
}

fn within_w0(got: f64, want: f64, what: &str) {
    assert!((got - want).abs() <= w0_i(want), "{what}: got {got:.12e}, want {want:.12e}, {:.3} W0", (got - want).abs() / w0_i(want));
}

/// The published rail's right-hand rate at the engine's time: the rail 1 ns
/// on, from a snapshot that is then put back.
fn rail_rate(e: &mut Engine, rail: usize) -> f64 {
    let (t, u) = (e.time(), e.voltages()[rail]);
    let snap = e.snapshot();
    e.advance(t + 1e-9).unwrap();
    let u1 = e.voltages()[rail];
    e.restore(&snap);
    (u1 - u) / 1e-9
}

/// E 5 V, r 1 ohm; a 10 uF rail capacitor (device 1, `esr`), R 1 kohm from
/// the rail (node 1) to x (node 2), C 1 uF from x to ground, and a switch
/// (device 5) putting 100 ohm (node 3) across the rail.
fn coupled_rc(esr: f64) -> Program {
    program(
        4,
        vec![battery(0, 1, 0, 5.0, 1.0), capacitor(1, 1, 0, 10e-6, esr), resistor(2, 1, 2, 1000.0), capacitor(3, 2, 0, 1e-6, 0.0), resistor(4, 3, 0, 100.0), switch(5, 1, 3, false)],
    )
}

/// The hand closed form of [`coupled_rc`], switch open: the rail is
/// algebraic, `u = E - r (u - vc) / R`, so `u = (E + (r/R) vc) / a`, `a = 1 +
/// r/R`, and `C vc' = (u - vc) / R` is one exponential with `tau' = (R + r)
/// C`: `vc = 5 (1 - e^(-t/tau'))`, `u' = (r/R)/a vc' = r/(R + r) 5/tau'
/// e^(-t/tau')`. Returns `(vc, u, u')`.
fn coupled_open(t: f64) -> (f64, f64, f64) {
    let (e, r, big_r, c) = (5.0, 1.0, 1000.0, 1e-6);
    let tau = (big_r + r) * c;
    let vc = e * (1.0 - (-t / tau).exp());
    let u = e - (e - vc) * r / (big_r + r);
    (vc, u, r / (big_r + r) * e / tau * (-t / tau).exp())
}

#[test]
fn a_cell_rail_capacitor_carries_c_du_dt_of_the_coupled_rc_and_of_the_switched_load() {
    // Before the switch, the hand closed form above: the rail capacitor reads
    // `C_r u'` with `u'` from `coupled_open` (5 uA at t = 0, falling with
    // tau' = 1.001 ms). After the switch closes at t1 = 0.5 ms the rail adds
    // `u / 100`: `a = 1 + r/R + r/100 = 1.011`, `u = (E + (r/R) vc)/a`, `vc'
    // = (u - vc)/(R C)`, so the right-hand rate at t1 is `(r/R)/a vc'(t1)`
    // with `vc(t1)` continuous from the open phase, and from then `vc`
    // settles on `vc_inf = E / (1 + r/100) = 4.950495` at the rate `lambda =
    // (1 - r/(R a))/(R C)`: `u'(t) = (r/R)/a (vc_inf - vc(t1)) lambda
    // e^(-lambda (t - t1))`. The jump of `u` itself at t1 (about -49 mV, the
    // unmodelled `r C_r` transient's) carries no published current: the
    // reading there is the right-hand one (README).
    let (e, r, big_r, c, c_r) = (5.0, 1.0, 1000.0, 1e-6, 10e-6);
    let mut eng = Engine::load(&coupled_rc(0.0)).unwrap();
    let t1 = 0.5e-3;
    for k in 1..=10 {
        let t = t1 * k as f64 / 10.0;
        eng.advance(t).unwrap();
        let (_, u, du) = coupled_open(t);
        assert!((eng.voltages()[1] - u).abs() <= 2.0 * crate::rails::RAIL_DRIFT_V, "the rail on its hand closed form at {t}");
        within_w0(eng.currents()[1], c_r * du, &format!("rail capacitor at {t} against C_r u' by hand"));
        within_w0(eng.currents()[1], c_r * rail_rate(&mut eng, 1), &format!("rail capacitor at {t} against C_r du/dt of the published rail"));
    }
    eng.set_params(&[(5, 0, 1.0)]).unwrap();
    let a = 1.0 + r / big_r + r / 100.0;
    let vc1 = coupled_open(t1).0;
    let vc_inf = e / (1.0 + r / 100.0);
    let lambda = (1.0 - r / (big_r * a)) / (big_r * c);
    let du = |t: f64| (r / big_r) / a * (vc_inf - vc1) * lambda * (-lambda * (t - t1)).exp();
    within_w0(eng.currents()[1], c_r * du(t1), "rail capacitor just after the switch, the right-hand rate");
    for k in 1..=10 {
        let t = t1 + 2e-3 * k as f64 / 10.0;
        eng.advance(t).unwrap();
        within_w0(eng.currents()[1], c_r * du(t), &format!("rail capacitor at {t}, switch closed, against C_r u' by hand"));
        within_w0(eng.currents()[1], c_r * rail_rate(&mut eng, 1), &format!("rail capacitor at {t} against C_r du/dt of the published rail"));
    }
}

/// KCL at the rail node of [`coupled_rc`]: what the cell delivers into its
/// `pos` (its published current is minus that) is the resistors' currents
/// off the rail, the rail capacitor's and the rail node's own shunt.
fn rail_kcl(eng: &Engine) -> (f64, f64) {
    let (v, i) = (eng.voltages(), eng.currents());
    let leaving = i[2] + i[1] + (i[5]) + static_part::NODE_RSHUNT_G * v[1];
    (-i[0], leaving)
}

#[test]
fn the_cells_current_is_its_loads_plus_its_rail_capacitors_at_every_published_instant() {
    // Before CAPI the rail capacitor published 0 and so did its share of the
    // cell's current: the cell read the loads alone, 5 uA (by hand at t = 0,
    // `C_r u'(0)`) short of what leaves the rail.
    for esr in [0.0, 0.05] {
        let mut eng = Engine::load(&coupled_rc(esr)).unwrap();
        let mut t = 0.0f64;
        while t < 2e-3 {
            if (t - 0.5e-3).abs() < 1e-15 {
                eng.set_params(&[(5, 0, 1.0)]).unwrap();
                let (cell, leaving) = rail_kcl(&eng);
                within_w0(cell, leaving, &format!("esr {esr}: KCL at the switch"));
            }
            t = (eng.next_event_time().min(t + 1e-4)).min(2e-3);
            eng.advance(t).unwrap();
            let (cell, leaving) = rail_kcl(&eng);
            within_w0(cell, leaving, &format!("esr {esr}: KCL at the rail node at {t}"));
            assert!(eng.currents()[1].abs() > 0.0 || t > 1.9e-3, "esr {esr}: the rail capacitor carries current at {t}");
        }
    }
}

#[test]
fn a_rail_capacitor_with_esr_follows_the_rail_as_one_without() {
    // The model takes the capacitor's own `esr C` lag out as it takes the
    // cell's `r C_r` (both are the rail capacitor's transient, which the
    // algebraic rail does not carry): with esr 0.05 ohm (`esr C_r` = 0.5 us)
    // the reading is still `C_r u'` of the hand closed form, at every stop,
    // where the staircase it replaced read `(u - vc)/esr` of the rail's last
    // move, ten times `C_r u'` at 10 us stops on fixture 77.
    let c_r = 10e-6;
    let mut eng = Engine::load(&coupled_rc(0.05)).unwrap();
    for k in 1..=50 {
        let t = 1e-5 * k as f64;
        eng.advance(t).unwrap();
        within_w0(eng.currents()[1], c_r * coupled_open(t).2, &format!("esr 0.05 at {t}"));
    }
}

#[test]
fn two_rail_capacitors_each_carry_their_own_c_du_dt_whichever_way_round_and_to_any_fixed_node() {
    // The board of `coupled_rc` with the 10 uF split: 4.7 uF rail to ground
    // (device 1), 3.3 uF ground to rail (device 6, so its `pins[0] - pins[1]`
    // falls as the rail rises: `-C u'`), 2 uF rail to node 4 (device 7),
    // which an ideal 1 V source (device 8) fixes: `d(u - 1)/dt = u'`. The
    // rail's equation does not see them (Rails point 7), so `u'` is
    // `coupled_open`'s, and each reads its own `C_k u'` signed by its pins.
    let p = program(
        5,
        vec![
            battery(0, 1, 0, 5.0, 1.0),
            capacitor(1, 1, 0, 4.7e-6, 0.0),
            resistor(2, 1, 2, 1000.0),
            capacitor(3, 2, 0, 1e-6, 0.0),
            resistor(4, 3, 0, 100.0),
            switch(5, 1, 3, false),
            capacitor(6, 0, 1, 3.3e-6, 0.0),
            capacitor(7, 1, 4, 2e-6, 0.0),
            source(8, 4, 0, 1.0),
        ],
    );
    let mut eng = Engine::load(&p).unwrap();
    for k in 1..=20 {
        let t = 5e-5 * k as f64;
        eng.advance(t).unwrap();
        let du = coupled_open(t).2;
        within_w0(eng.currents()[1], 4.7e-6 * du, &format!("4.7 uF at {t}"));
        within_w0(eng.currents()[6], -3.3e-6 * du, &format!("3.3 uF, reversed, at {t}"));
        within_w0(eng.currents()[7], 2e-6 * du, &format!("2 uF to the 1 V node at {t}"));
        // KCL at the rail: the cell feeds the 1 kohm, the three capacitors and
        // the rail's shunt (the switch is open).
        let (v, i) = (eng.voltages(), eng.currents());
        let leaving = i[2] + i[1] - i[6] + i[7] + static_part::NODE_RSHUNT_G * v[1];
        within_w0(-i[0], leaving, &format!("KCL at the rail at {t}"));
    }
}

#[test]
fn a_rail_capacitor_on_a_rail_that_holds_between_its_solves_reads_zero_after_each_step() {
    // E 5, r 1, 10 uF across the rail, 1 kohm across the rail and a switch
    // (1 mohm closed) putting 100 ohm across it: nothing on the rail moves
    // between solves, so the rail holds at `u = 5 R/(R + r)` (4.995005), then
    // at `5 Rl/(Rl + r)`, `Rl = 1000 || 100.001` (4.945599). The right-hand
    // rate at every instant, the step's included, is 0, so the capacitor
    // reads exactly 0 (it did before CAPI too: this pins that it still does).
    let p = program(4, vec![battery(0, 1, 0, 5.0, 1.0), capacitor(1, 1, 0, 10e-6, 0.0), resistor(2, 1, 0, 1000.0), resistor(3, 2, 0, 100.0), switch(4, 1, 2, false)]);
    let mut eng = Engine::load(&p).unwrap();
    eng.advance(1e-3).unwrap();
    assert!((eng.voltages()[1] - 5.0 * 1000.0 / 1001.0).abs() < 1e-9, "the rail before the step");
    assert_eq!(eng.currents()[1], 0.0, "the held rail's capacitor");
    eng.set_params(&[(4, 0, 1.0)]).unwrap();
    let rl = 1000.0 * 100.001 / 1100.001;
    assert!((eng.voltages()[1] - 5.0 * rl / (rl + 1.0)).abs() < 1e-9, "the rail after the step");
    assert_eq!(eng.currents()[1], 0.0, "the right-hand rate at the step");
    eng.advance(2e-3).unwrap();
    assert_eq!(eng.currents()[1], 0.0, "the held rail's capacitor after the step");
}

#[test]
fn a_rail_capacitor_on_a_rail_stepped_draws_move_reads_the_rate_of_its_last_move() {
    // E 5, r 1, 10 uF across the rail; 220 ohm from the rail to node 2, an
    // LED and 10 uF from node 2 to ground: the LED's partition is stepped
    // (a junction beside a capacitor), so its draws move the rail at each of
    // its commits and nothing models the rail between them (ruling (a): the
    // published rail is linear between its solves). At each instant the
    // capacitor reads `C_r (u_k - u_j) / (t_k - t_j)` of the published rail,
    // `j` the last earlier instant at which the rail moved, when the rail
    // moved at this instant, else 0; the cell's current closes KCL with it.
    let p = program(3, vec![battery(0, 1, 0, 5.0, 1.0), capacitor(1, 1, 0, 10e-6, 0.0), resistor(2, 1, 2, 220.0), led(3, 2, 0), capacitor(4, 2, 0, 10e-6, 0.0)]);
    let c_r = 10e-6;
    let mut eng = Engine::load(&p).unwrap();
    let (mut last_move, mut u_prev, mut t) = ((0.0f64, eng.voltages()[1]), eng.voltages()[1], 0.0f64);
    let mut moved_instants = 0;
    while t < 20e-3 {
        t = eng.next_event_time().min(20e-3);
        eng.advance(t).unwrap();
        let u = eng.voltages()[1];
        let want = if u.to_bits() != u_prev.to_bits() {
            let rate = (u - last_move.1) / (t - last_move.0);
            last_move = (t, u);
            moved_instants += 1;
            c_r * rate
        } else {
            0.0
        };
        u_prev = u;
        within_w0(eng.currents()[1], want, &format!("rail capacitor at {t}"));
        let (v, i) = (eng.voltages(), eng.currents());
        within_w0(-i[0], i[2] + i[1] + static_part::NODE_RSHUNT_G * v[1], &format!("KCL at the rail at {t}"));
    }
    assert!(moved_instants > 10, "the stepped draws moved the rail at {moved_instants} instants");
}

#[test]
fn exact_mode_carries_the_rail_capacitor_in_the_rails_partition_and_closes_kcl() {
    // Exact mode makes the rail a free node, so the rail capacitor is a
    // capacitor of the rail's stepped partition and publishes its companion
    // current (README, exact mode), with the `r C_r` transient default mode
    // leaves out: from load (the capacitor discharged, W1b point 5) the rail
    // charges it through r. With C_r = 1 uF that transient is `r C_r` = 1 us,
    // gone (e^-200) by 0.2 ms, after which the reading is the hand `C_r u'`
    // of `coupled_open` to first order in `r C_r / tau'` (1e-3) and the
    // stepped integrator's truncation: its steps grow as the RC settles, and
    // the companion over a step reads its rate to the step's own error (2.4e-2
    // at 1.6 ms, README's open item on exact mode's coarse step accuracy),
    // so the hand bound is 5e-2. The cell, a Norton branch,
    // closes KCL with it within W0 at every stop.
    let mut p = coupled_rc(0.0);
    p.devices[1].params[0] = 1e-6;
    let mut eng = Engine::load(&exact(p)).unwrap();
    for k in 1..=10 {
        let t = 0.2e-3 * k as f64;
        eng.advance(t).unwrap();
        let want = 1e-6 * coupled_open(t).2;
        let got = eng.currents()[1];
        assert!((got - want).abs() <= 1e-9 + 5e-2 * want.abs(), "exact mode at {t}: {got:e} against {want:e}");
        let (cell, leaving) = rail_kcl(&eng);
        within_w0(cell, leaving, &format!("exact mode, KCL at the rail at {t}"));
    }
}

#[test]
fn the_charge_of_a_cell_rail_capacitor_is_c_du_of_its_rail_between_instants() {
    // Charge (README): the integral of the published current. On the coupled
    // path that is `C_r (u(t1) - u(t0))` over a span with no jump of the
    // rail; by hand from `coupled_open`, 10 uF (u(1 ms) - u(0.2 ms)). The
    // cell's charge is minus what leaves the rail, the capacitor's included.
    let mut eng = Engine::load(&coupled_rc(0.0)).unwrap();
    eng.advance(0.2e-3).unwrap();
    eng.charge_arm(&[1, 0, 2]).unwrap();
    for k in 1..=8 {
        eng.advance(0.2e-3 + 1e-4 * k as f64).unwrap();
    }
    let out = eng.take_charge().to_vec();
    let (t0, t1) = (out[0], out[1]);
    let want = 10e-6 * (coupled_open(t1).1 - coupled_open(t0).1);
    let span = t1 - t0;
    assert!((out[2] - want).abs() <= 1e-9 * span + 1e-3 * want.abs(), "rail capacitor charge {:e} against {want:e}", out[2]);
    // The cell against the 1 kohm and the capacitor (the shunt's charge is
    // below 1e-15 C here).
    let leaving = out[2] + out[4];
    assert!((-out[3] - leaving).abs() <= 1e-9 * span + 1e-3 * leaving.abs(), "the cell's charge {:e} against {leaving:e}", -out[3]);
}

// ---- CAPERR: a Norton ladder's capacitor currents (3.3 V behind `rs` into r1 || c1, then r2 into c2) ----

/// The ladder: source 3.3 V (node 1) behind `rs` into node 2 (1 kohm and 1 uF
/// to ground), 1 kohm on to node 3 (1 uF to ground). Devices 4 and 5 are the
/// capacitors.
fn ladder(rs: f64) -> Program {
    program(4, vec![source(0, 1, 0, 3.3), resistor(1, 1, 2, rs), resistor(2, 2, 0, 1000.0), resistor(3, 2, 3, 1000.0), capacitor(4, 2, 0, 1e-6, 0.0), capacitor(5, 3, 0, 1e-6, 0.0)])
}

/// The ladder's two capacitor currents `(c1, c2)` at `t` after the source
/// steps on, from the closed form of its two real poles: `C x' = M x + f`
/// with `x = (vA, vB)`, `x(0) = 0`, `x = xs + a1 e1 exp(l1 t) + a2 e2 exp(l2
/// t)`, `M e = l e`.
fn ladder_currents(rs: f64, t: f64) -> (f64, f64) {
    let c = 1e-6;
    let (m11, m12, m21, m22) = (-(1.0 / rs + 2.0 / 1000.0) / c, 1e-3 / c, 1e-3 / c, -1e-3 / c);
    let f1 = 3.3 / (rs * c);
    let (tr, det) = (m11 + m22, m11 * m22 - m12 * m21);
    let disc = (tr * tr / 4.0 - det).sqrt();
    let l = [tr / 2.0 + disc, tr / 2.0 - disc];
    let xs = [-f1 * m22 / det, f1 * m21 / det];
    let e = [[m12, l[0] - m11], [m12, l[1] - m11]];
    let d = e[0][0] * e[1][1] - e[1][0] * e[0][1];
    let a = [(-xs[0] * e[1][1] + e[1][0] * xs[1]) / d, (-e[0][0] * xs[1] + xs[0] * e[0][1]) / d];
    let dx = |j: usize| (0..2).map(|k| a[k] * e[k][j] * l[k] * (l[k] * t).exp()).sum::<f64>();
    (c * dx(0), c * dx(1))
}

#[test]
fn a_host_stop_a_few_femtoseconds_after_the_last_commit_reads_the_capacitor_current_the_stop_before_it_did() {
    // The ladder, stepped (the trapezoid after its first step). A host stop
    // d after the stop before it is a step of d: below the floor, where the
    // companion's current is `2 C / d` times a difference of voltages that
    // agree to their last bits: at d = 5.9 fs on 1 uF, 1.7e8 S times the
    // rounding of 3.1 V is 7.6e-8 A, 4 W0 on c1's 16 uA at 2.33 ms (the
    // 10 ns host schedule's accumulated clock lands there). The state does
    // not move in d, so the reading must not either (the closed form's own
    // move over d is 1e-13 A); and the noisy current read as the step's
    // estimate failed the floor step's check, which put the partition on
    // backward Euler for good (stat 27, R81 (1)).
    let t_end = 2.33e-3;
    for d in [1e-13, 1e-14, 5.86e-15, 1e-15] {
        let mut eng = Engine::load(&ladder(50.0)).unwrap();
        for t in [0.05e-3, 0.13e-3, 0.31e-3, 0.52e-3, 0.77e-3, 1.21e-3, 1.55e-3, t_end - d] {
            eng.advance(t).unwrap();
        }
        let before = [eng.currents()[4], eng.currents()[5]];
        eng.advance(t_end).unwrap();
        for (k, name) in [(4, "c1"), (5, "c2")] {
            let (b, a) = (before[k - 4], eng.currents()[k]);
            assert!((a - b).abs() <= 0.1 * w0_i(b), "{name} {d:e} s after the last stop: {a:.12e} against {b:.12e}, {:.2} W0", (a - b).abs() / w0_i(b));
        }
        assert_eq!(eng.stat(27), 0.0, "{d:e} s stop put the ladder on backward Euler");
    }
}

#[test]
fn the_ladder_read_every_10_ns_for_2_33_ms_keeps_c1_within_w0_of_its_closed_form() {
    // The probe's schedule: `now += 1e-8` accumulates to 5.9 fs short of
    // 2.33 ms after 233,000 stops, so the last stop is a sliver (3.7 W0 off
    // before the sliver fix); every stop before it is a truncated trapezoid
    // step and reads the closed form to 1e-3 W0.
    let t_end = 2.33e-3;
    let mut eng = Engine::load(&ladder(50.0)).unwrap();
    let mut now = 0.0f64;
    while now < t_end {
        now = (now + 1e-8).min(t_end);
        eng.advance(now).unwrap();
    }
    within_w0(eng.currents()[4], ladder_currents(50.0, t_end).0, "c1 at 2.33 ms after 233,001 stops of 10 ns");
    within_w0(eng.currents()[5], ladder_currents(50.0, t_end).1, "c2 at 2.33 ms after 233,001 stops of 10 ns");
}

#[test]
#[should_panic(expected = "off the closed form")]
// CAPERR's open half, expected until a candidate lands: seven stops from 0.05
// to 2.33 ms read c1 up to 3.0 W0 off (0.31 ms) and 1.2 at the end: the
// trapezoid's local errors, each inside the step control's own 10 nA and
// 1e-3, add up over the five or six long steps the transient takes.
fn the_ladder_read_in_a_few_uneven_stops_keeps_its_capacitor_currents_within_w0_of_the_closed_form() {
    let mut eng = Engine::load(&ladder(50.0)).unwrap();
    let mut worst = 0.0f64;
    for t in [0.05e-3, 0.13e-3, 0.31e-3, 0.52e-3, 0.77e-3, 1.21e-3, 1.55e-3, 2.33e-3] {
        eng.advance(t).unwrap();
        let (i1, i2) = ladder_currents(50.0, t);
        worst = worst.max((eng.currents()[4] - i1).abs() / w0_i(i1)).max((eng.currents()[5] - i2).abs() / w0_i(i2));
    }
    assert!(worst <= 1.0, "the ladder's capacitor currents read up to {worst:.2} W0 off the closed form");
}

#[test]
#[should_panic(expected = "off the closed form")]
// The same open half on a second board (a verifier's push_dpdt, reduced): two
// RC stages from 5 V (1 kohm into 1 uF, 1 kohm into 2 uF) joined into one
// stepped partition by a 1 Gohm resistor, the host stopping every 1 ms.
// Voltages are inside W0; the capacitors' currents read up to 6.1 W0 off the
// closed form, 15 of 24 readings past W0, where either stage alone is exact.
// Expected until the trapezoid's local errors are bounded where a current is
// published.
fn two_rc_stages_joined_by_a_gigaohm_keep_their_capacitor_currents_within_w0_of_the_closed_form() {
    let (r1, c1, r2, c2, rb) = (1000.0, 1e-6, 1000.0, 2e-6, 1e9);
    let p = program(5, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, r1), capacitor(2, 2, 0, c1, 0.0), resistor(3, 1, 3, r2), capacitor(4, 3, 0, c2, 0.0), resistor(5, 2, 3, rb)]);
    // C1 a' = (5 - a) / r1 - (a - b) / rb, C2 b' = (5 - b) / r2 + (a - b) / rb.
    let (m11, m12, m21, m22) = (-(1.0 / r1 + 1.0 / rb) / c1, 1.0 / (rb * c1), 1.0 / (rb * c2), -(1.0 / r2 + 1.0 / rb) / c2);
    let (f1, f2) = (5.0 / (r1 * c1), 5.0 / (r2 * c2));
    let (tr, det) = (m11 + m22, m11 * m22 - m12 * m21);
    let disc = (tr * tr / 4.0 - det).sqrt();
    let l = [tr / 2.0 + disc, tr / 2.0 - disc];
    let xs = [(-f1 * m22 + m12 * f2) / det, (-m11 * f2 + m21 * f1) / det];
    let e = [[m12, l[0] - m11], [m12, l[1] - m11]];
    let d = e[0][0] * e[1][1] - e[1][0] * e[0][1];
    let a = [(-xs[0] * e[1][1] + e[1][0] * xs[1]) / d, (-e[0][0] * xs[1] + xs[0] * e[0][1]) / d];
    let dx = |j: usize, t: f64| (0..2).map(|k| a[k] * e[k][j] * l[k] * (l[k] * t).exp()).sum::<f64>();
    let mut eng = Engine::load(&p).unwrap();
    let mut worst = 0.0f64;
    for k in 1..=12 {
        let t = k as f64 * 1e-3;
        eng.advance(t).unwrap();
        for (j, (c, dev)) in [(c1, 2), (c2, 4)].into_iter().enumerate() {
            let want = c * dx(j, t);
            worst = worst.max((eng.currents()[dev] - want).abs() / w0_i(want));
        }
    }
    assert!(worst <= 1.0, "the stages' capacitor currents read up to {worst:.2} W0 off the closed form");
}
