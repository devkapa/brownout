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

// ---- CAPERR2: the accumulation half. Every capacitor current within W0 of the closed form at every accepted step and every host stop ----

/// The closed form of `C x' = M x + f` from `x(0) = 0` for two states: the
/// state and its derivative `j` at `t`, `x = xs + sum a_k e_k exp(l_k t)`.
/// Each eigenvector is taken from the row of `M - l I` that is larger, so a
/// weakly coupled pair (1 Gohm between two stages) keeps its precision.
struct Pair {
    l: [f64; 2],
    e: [[f64; 2]; 2],
    a: [f64; 2],
    xs: [f64; 2],
}

impl Pair {
    fn new(m: [[f64; 2]; 2], f: [f64; 2]) -> Pair {
        let (tr, det) = (m[0][0] + m[1][1], m[0][0] * m[1][1] - m[0][1] * m[1][0]);
        let disc = (tr * tr / 4.0 - det).sqrt();
        let l = [tr / 2.0 + disc, tr / 2.0 - disc];
        let xs = [(-f[0] * m[1][1] + m[0][1] * f[1]) / det, (-m[0][0] * f[1] + m[1][0] * f[0]) / det];
        let e = l.map(|lk| {
            let (u, w) = ([m[0][1], lk - m[0][0]], [lk - m[1][1], m[1][0]]);
            if u[0].hypot(u[1]) >= w[0].hypot(w[1]) { u } else { w }
        });
        let d = e[0][0] * e[1][1] - e[1][0] * e[0][1];
        let a = [(-xs[0] * e[1][1] + e[1][0] * xs[1]) / d, (-e[0][0] * xs[1] + xs[0] * e[0][1]) / d];
        Pair { l, e, a, xs }
    }
    fn x(&self, j: usize, t: f64) -> f64 {
        self.xs[j] + (0..2).map(|k| self.a[k] * self.e[k][j] * (self.l[k] * t).exp()).sum::<f64>()
    }
    fn dx(&self, j: usize, t: f64) -> f64 {
        (0..2).map(|k| self.a[k] * self.e[k][j] * self.l[k] * (self.l[k] * t).exp()).sum::<f64>()
    }
}

/// The ladder's closed form: `(vA, vB)` and the capacitor currents.
fn ladder_pair(rs: f64) -> Pair {
    let c = 1e-6;
    Pair::new([[-(1.0 / rs + 2.0 / 1000.0), 1e-3], [1e-3, -1e-3]].map(|r| r.map(|v| v / c)), [3.3 / (rs * c), 0.0])
}

/// Two RC stages from `v` (`r1` into `c1`, `r2` into `c2`) joined by `rb`:
/// devices 2 and 4 are the capacitors, node 2 and 3 their voltages.
fn stages(v: f64, r1: f64, c1: f64, r2: f64, c2: f64, rb: f64) -> (Program, Pair) {
    let p = program(5, vec![source(0, 1, 0, v), resistor(1, 1, 2, r1), capacitor(2, 2, 0, c1, 0.0), resistor(3, 1, 3, r2), capacitor(4, 3, 0, c2, 0.0), resistor(5, 2, 3, rb)]);
    let m = [[-(1.0 / r1 + 1.0 / rb) / c1, 1.0 / (rb * c1)], [1.0 / (rb * c2), -(1.0 / r2 + 1.0 / rb) / c2]];
    (p, Pair::new(m, [v / (r1 * c1), v / (r2 * c2)]))
}

/// Advances `p` through `stops`; every accepted step of the run and every
/// stop reads its two capacitors (devices `dev`, in the order the partition
/// holds them) against `closed`, in W0 of the current. Returns `(t, [e0,
/// e1], at_a_stop)`, in time order, and the voltages' worst W0 (100 uV + 1e-3).
fn read_against(p: &Program, closed: &Pair, caps: [f64; 2], dev: [usize; 2], stops: &[f64]) -> (Vec<(f64, [f64; 2], bool)>, f64) {
    let mut eng = Engine::load(p).unwrap();
    stepped::ACCEPTED.with(|a| *a.borrow_mut() = Some(Vec::new()));
    let mut out = vec![];
    let mut vworst = 0.0f64;
    for &t in stops {
        eng.advance(t).unwrap();
        let mut e = [0.0; 2];
        for j in 0..2 {
            let want = caps[j] * closed.dx(j, t);
            e[j] = (eng.currents()[dev[j]] - want) / w0_i(want);
            let v = closed.x(j, t);
            vworst = vworst.max((eng.voltages()[2 + j] - v).abs() / (1e-4 + 1e-3 * v.abs()));
        }
        out.push((t, e, true));
    }
    let log = stepped::ACCEPTED.with(|a| a.borrow_mut().take().unwrap());
    for (t, i) in log {
        let mut e = [0.0; 2];
        for j in 0..2 {
            let want = caps[j] * closed.dx(j, t);
            e[j] = (i[j] - want) / w0_i(want);
        }
        out.push((t, e, false));
    }
    out.sort_by(|a, b| a.0.total_cmp(&b.0));
    (out, vworst)
}

fn worst_of(r: &[(f64, [f64; 2], bool)]) -> (f64, f64, bool) {
    r.iter().fold((0.0, 0.0, false), |w, &(t, e, stop)| { let m = e[0].abs().max(e[1].abs()); if m > w.0 { (m, t, stop) } else { w } })
}

/// The ladder in seven uneven stops to 2.33 ms, every capacitor current at
/// every accepted step and host stop against the closed form: the worst
/// `(W0, t, at a stop)` and the worst node, in W0.
fn ladder_worst(rs: f64) -> ((f64, f64, bool), f64) {
    let stops = [0.05e-3, 0.13e-3, 0.31e-3, 0.52e-3, 0.77e-3, 1.21e-3, 1.55e-3, 2.33e-3];
    let (r, vworst) = read_against(&ladder(rs), &ladder_pair(rs), [1e-6, 1e-6], [4, 5], &stops);
    (worst_of(&r), vworst)
}

#[test]
fn the_ladder_at_50_and_500_ohm_keeps_both_capacitor_currents_within_w0_at_every_accepted_step_and_stop() {
    // Before the extrapolation c1 read 3.9 W0 off at an accepted step with
    // 50 ohm, 1.45 with 500, and 1.2 at the 2.33 ms stop: the trapezoid's
    // local errors, each inside the step control's own 10 nA and 1e-3, add up
    // over the five or six long steps the transient takes.
    for rs in [50.0, 500.0] {
        let ((w, t, stop), vworst) = ladder_worst(rs);
        assert!(vworst <= 1.0, "rs {rs}: a node read {vworst:.2} W0 off its closed form");
        assert!(w <= 1.0, "rs {rs}: a capacitor current reads {w:.2} W0 off the closed form at t = {t:e} ({})", if stop { "a host stop" } else { "an accepted step" });
    }
}

#[test]
// CAPERR2's open half at 5 ohm (R136's limit of 10 on the rate bound times the
// step), closed by CAPERR4: past the guard the step is the extrapolated TR-BDF2.
fn the_ladder_at_5_ohm_keeps_both_capacitor_currents_within_w0_at_every_accepted_step_and_stop() {
    let ((w, t, _), _) = ladder_worst(5.0);
    assert!(w <= 1.0, "rs 5: a capacitor current reads {w:.2} W0 off the closed form at t = {t:e}");
}

#[test]
fn two_rc_stages_joined_by_a_gigaohm_keep_their_capacitor_currents_within_w0_at_every_accepted_step_and_stop() {
    // A verifier's push_dpdt, reduced: 1 kohm into 1 uF and 1 kohm into 2 uF
    // from 5 V, joined into one stepped partition by 1 Gohm, the host
    // stopping every 1 ms. Before the extrapolation the capacitors' currents
    // read up to 6.1 W0 off, 15 of 24 readings past W0, where either stage
    // alone is exact.
    let (p, closed) = stages(5.0, 1000.0, 1e-6, 1000.0, 2e-6, 1e9);
    let stops: Vec<f64> = (1..=12).map(|k| k as f64 * 1e-3).collect();
    stepped::NO_EXTRAP.with(|c| c.set(true));
    let (plain, _) = read_against(&p, &closed, [1e-6, 2e-6], [2, 4], &stops);
    stepped::NO_EXTRAP.with(|c| c.set(false));
    let (r, vworst) = read_against(&p, &closed, [1e-6, 2e-6], [2, 4], &stops);
    let (w, t, stop) = worst_of(&r);
    eprintln!("two stages: plain {:.3} W0, extrapolated {w:.3} W0", worst_of(&plain).0);
    assert!(vworst <= 1.0, "a node reads {vworst:.2} W0 off its closed form");
    assert!(w <= 1.0, "a capacitor current reads {w:.2} W0 off the closed form at t = {t:e} ({}, {} readings)", if stop { "a host stop" } else { "an accepted step" }, r.len());
}

#[test]
fn a_fast_mode_at_1_to_1000_steps_of_its_rate_never_rings_with_growing_amplitude_nor_reads_worse_than_the_plain_trapezoid() {
    // One slow stage (10 kohm, 1 uF: 100 /s) and one fast (1 uF, `r` chosen
    // for a rate of P / 1 ms) joined by 1 Gohm, the host stopping every 1 ms
    // after the source's step (so the partition's first step is a coarse
    // backward Euler one): the fast rate times the step runs from 1 to 1000
    // by the stage's resistor. The extrapolated trapezoid's amplification
    // tends to 5/3 for a very stiff mode, so each reading must stay within
    // W0 of the closed form, or no worse than the plain trapezoid's own
    // (`NO_EXTRAP`), whichever is larger, and no run of readings may swing
    // with an amplitude that grows from one to the next.
    for prod in [1.0, 5.0, 15.0, 25.0, 50.0, 200.0, 1000.0] {
        let t_stop = 1e-3;
        let rf = t_stop / (prod * 1e-6);
        let (p, closed) = stages(5.0, rf, 1e-6, 10_000.0, 1e-6, 1e9);
        let stops: Vec<f64> = (1..=12).map(|k| k as f64 * t_stop).collect();
        stepped::NO_EXTRAP.with(|c| c.set(true));
        let (plain, _) = read_against(&p, &closed, [1e-6, 1e-6], [2, 4], &stops);
        stepped::NO_EXTRAP.with(|c| c.set(false));
        stepped::EXTRAPOLATED.with(|c| c.set(0));
        let (got, vworst) = read_against(&p, &closed, [1e-6, 1e-6], [2, 4], &stops);
        let (wp, ..) = worst_of(&plain);
        let (w, t, _) = worst_of(&got);
        eprintln!("P {prod}: plain {wp:.3} W0, extrapolated {w:.3} W0 (volt {vworst:.3}), extrapolated steps {}", stepped::EXTRAPOLATED.with(|c| c.get()));
        assert!(vworst <= 1.0, "P {prod}: a node reads {vworst:.2} W0 off its closed form");
        assert!(w <= wp.max(1.0) * (1.0 + 1e-9), "P {prod}: a capacitor current reads {w:.2} W0 off at t = {t:e}, the plain trapezoid {wp:.2}");
        // Growth: in time order, three readings of one capacitor whose errors
        // alternate in sign, each larger than the last by a fifth, past W0.
        for j in 0..2 {
            let e: Vec<f64> = got.iter().map(|g| g.1[j]).collect();
            for k in 2..e.len() {
                let ring = e[k - 2] * e[k - 1] < 0.0 && e[k - 1] * e[k] < 0.0 && e[k - 1].abs() > 1.2 * e[k - 2].abs() && e[k].abs() > 1.2 * e[k - 1].abs() && e[k].abs() > 1.0;
                assert!(!ring, "P {prod}: capacitor {j} rings with growing amplitude at reading {k}: {:.2}, {:.2}, {:.2} W0", e[k - 2], e[k - 1], e[k]);
            }
        }
    }
}

// ---- CAPERR2 (R136): the extrapolation's stability guard, and what it leaves alone ----

/// A source behind `r` into `c` to ground (the fast stage), and a slow stage
/// (1 Mohm into 1 uF, a second of time constant) joined to it by 1 Gohm so
/// the partition keeps stepping. Returns the board and the fast rate `2 G / c`
/// as `stepped::fastest_rate_bound` bounds it (`G` the node's conductances,
/// the 1e-12 S shunt every node carries and the joining 1 nS included).
fn rc(r: f64, c: f64) -> (Program, f64) {
    let p = program(5, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, r), capacitor(2, 2, 0, c, 0.0), resistor(3, 1, 3, 1e6), capacitor(4, 3, 0, 1e-6, 0.0), resistor(5, 2, 3, 1e9)]);
    (p, 2.0 * (1.0 / r + 1e-9 + 1e-12) / c)
}

/// Runs `p` 40 stops of 50 us, then takes one step of
/// exactly `h` (a host stop `h` on) and returns how many extrapolated
/// trapezoid steps and how many extrapolated TR-BDF2 steps ended within it.
fn extrapolated_by_a_step_of(p: &Program, h: f64) -> (u64, u64) {
    let mut eng = Engine::load(p).unwrap();
    for k in 1..=40 {
        eng.advance(k as f64 * 5e-5).unwrap();
    }
    stepped::EXTRAP_LOG.with(|l| l.borrow_mut().clear());
    let t0 = eng.time();
    eng.advance(t0 + h).unwrap();
    let log = stepped::EXTRAP_LOG.with(|l| l.borrow().clone());
    let within = |trbdf2: bool| log.iter().filter(|&&(t, b)| b == trbdf2 && t > t0 && t <= t0 + h * (1.0 + 1e-9)).count() as u64;
    (within(false), within(true))
}

#[test]
fn the_extrapolation_takes_a_step_whose_rate_bound_times_h_is_9_9_and_leaves_one_at_10_1() {
    // The guard: the fastest rate bounded by Gershgorin (`2 G_ii / c_i`) times
    // the step is at most 10. On one RC with the source settled, a host stop
    // `h` after the last commit is one accepted step of exactly `h`. Past it
    // (CAPERR4) the step is the extrapolated TR-BDF2 instead, one or the other.
    for (r, c) in [(1000.0, 1e-8), (100.0, 1e-7), (2.2e4, 1e-10)] {
        let (p, rate) = rc(r, c);
        stepped::EXTRAPOLATED.with(|c| c.set(0));
        assert_eq!(extrapolated_by_a_step_of(&p, 9.9 / rate), (1, 0), "R {r} C {c}: a step of 9.9 / rate");
        assert_eq!(extrapolated_by_a_step_of(&p, 10.1 / rate), (0, 1), "R {r} C {c}: a step of 10.1 / rate");
    }
}

/// Every stop's readings (voltages and currents), bit for bit.
fn run_bits(p: &Program, stops: &[f64]) -> Vec<u64> {
    let mut eng = Engine::load(p).unwrap();
    let mut out = vec![];
    for &t in stops {
        eng.advance(t).unwrap();
        out.extend(eng.voltages().iter().chain(eng.currents()).map(|x| x.to_bits()));
    }
    out
}

#[test]
fn a_partition_with_no_rate_bound_is_bit_for_bit_the_plain_trapezoid() {
    let stops: Vec<f64> = (1..=30).map(|k| k as f64 * 0.37e-3).collect();
    // (a) A capacitor between two free nodes has no diagonal C: no bound.
    let floating = program(4, vec![source(0, 1, 0, 5.0), resistor(1, 1, 2, 1000.0), capacitor(2, 2, 3, 1e-6, 0.0), resistor(3, 3, 0, 2200.0), resistor(4, 2, 0, 4700.0)]);
    for (name, p) in [("a floating capacitor", &floating)] {
        stepped::NO_EXTRAP.with(|c| c.set(true));
        let plain = run_bits(p, &stops);
        stepped::NO_EXTRAP.with(|c| c.set(false));
        stepped::EXTRAPOLATED.with(|c| c.set(0));
        let got = run_bits(p, &stops);
        assert_eq!(stepped::EXTRAPOLATED.with(|c| c.get()), 0, "{name}: a step was extrapolated");
        assert!(got == plain, "{name}: a reading differs from the plain trapezoid's");
    }
}

#[test]
fn a_snapshot_a_rollback_and_a_carried_reload_need_nothing_the_extrapolation_adds() {
    // The extrapolation commits the state a step already carries (x, vc, i): no
    // field is added, so a snapshot put back replays bit for bit, and the carry
    // buffer is the length it was. At 50 ohm the replayed span takes extrapolated
    // trapezoid steps, at 5 ohm (CAPERR4) extrapolated TR-BDF2 ones.
    for (rs, trbdf2) in [(50.0, false), (5.0, true)] {
        let p = ladder(rs);
        let counter = || if trbdf2 { stepped::EXTRAP_TRBDF2.with(|c| c.get()) } else { stepped::EXTRAPOLATED.with(|c| c.get()) };
        let stops = [0.2e-3, 0.5e-3, 0.9e-3, 1.4e-3, 2.0e-3, 2.6e-3];
        let mut eng = Engine::load(&p).unwrap();
        eng.advance(stops[1]).unwrap();
        let base = counter();
        let snap = eng.snapshot();
        let mut first = vec![];
        for &t in &stops[2..] {
            eng.advance(t).unwrap();
            first.extend(eng.voltages().iter().chain(eng.currents()).map(|x| x.to_bits()));
        }
        assert!(counter() > base, "rs {rs}: the replayed span took no extrapolated step");
        eng.restore(&snap);
        let mut second = vec![];
        for &t in &stops[2..] {
            eng.advance(t).unwrap();
            second.extend(eng.voltages().iter().chain(eng.currents()).map(|x| x.to_bits()));
        }
        assert!(first == second, "rs {rs}: a restored snapshot replayed differently");
        let carried = eng.carry_take().unwrap().len();
        stepped::NO_EXTRAP.with(|c| c.set(true));
        let mut plain = Engine::load(&p).unwrap();
        for &t in &stops {
            plain.advance(t).unwrap();
        }
        assert_eq!(plain.carry_take().unwrap().len(), carried, "rs {rs}: the carry buffer's length moved");
        stepped::NO_EXTRAP.with(|c| c.set(false));
    }
}


// ---- CAPERR3: the charge bill of an extrapolated step ----

#[test]
fn an_extrapolated_ladders_charge_bill_is_c_dvc_of_what_it_published_and_a_failed_advance_after_one_changes_nothing() {
    for (rs, trbdf2) in [(50.0, false), (5.0, true)] {
        extrapolated_ladders_bill(rs, trbdf2);
    }
}

/// The bill of the ladder at `rs`, whose long steps are extrapolated trapezoid
/// steps (`trbdf2` false) or extrapolated TR-BDF2 ones (CAPERR4).
fn extrapolated_ladders_bill(rs: f64, trbdf2: bool) {
    // The ladder at 50 ohm takes extrapolated steps (the guard passes its long ones); every device's
    // bill is the quadrature `(4 fine - coarse) / 3` of its points, so each capacitor's is `C dvc` of the
    // voltages published, the resistors' meet KCL at both nodes, and a failed advance to 2.0 ms
    // (Newton forced to give up) after one of those steps leaves the run, readings and bills, as one
    // that never failed.
    // An LED on an RC, a second partition, is the one whose Newton gives up: the instant rolls back
    // whole, the ladder's committed step with it.
    let mut p = ladder(rs);
    p.node_count = 6;
    p.devices.extend([source(6, 4, 0, 5.0), resistor(7, 4, 5, 1000.0), capacitor(8, 5, 0, 1e-6, 0.0), led(9, 5, 0)]);
    let all: Vec<u32> = (0..p.devices.len() as u32).collect();
    let stops = [0.2e-3, 0.5e-3, 0.9e-3, 1.4e-3, 2.0e-3, 2.6e-3];
    let run = |fail: bool| {
        let mut e = Engine::load(&p).unwrap();
        e.charge_arm(&all).unwrap();
        stepped::EXTRAPOLATED.with(|c| c.set(0));
        stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let mut q: Vec<f64> = Vec::new();
        let mut bits = Vec::new();
        for (k, &t) in stops.iter().enumerate() {
            if fail && k == 4 {
                let _guard = super::newton::hooks::force_unconverged(u32::MAX);
                assert!(e.advance(t).is_err(), "the forced failure");
                let b = e.take_charge().to_vec();
                let slots = b.len() - 2 - 2 * all.len();
                if q.is_empty() {
                    q = vec![0.0; slots];
                }
                b[2..2 + slots].iter().enumerate().for_each(|(i, x)| q[i] += x);
            }
            e.advance(t).unwrap();
            let b = e.take_charge().to_vec();
            let slots = b.len() - 2 - 2 * all.len();
            if q.is_empty() {
                q = vec![0.0; slots];
            }
            b[2..2 + slots].iter().enumerate().for_each(|(i, x)| q[i] += x);
            bits.extend(e.voltages().iter().chain(e.currents()).map(|x| x.to_bits()));
        }
        let taken = if trbdf2 { stepped::EXTRAP_TRBDF2.with(|c| c.get()) } else { stepped::EXTRAPOLATED.with(|c| c.get()) };
        assert!(taken > 0, "rs {rs}: no extrapolated step was taken");
        (e, q, bits)
    };
    let (e, q, bits) = run(false);
    let (v2, v3) = (e.voltages()[2], e.voltages()[3]);
    let close = |what: &str, got: f64, want: f64, tol: f64| assert!((got - want).abs() <= tol, "{what}: {got:e} against {want:e} (off {:e}, allowed {tol:e})", (got - want).abs());
    close("c1", q[4], 1e-6 * v2, 1e-12 * 1e-6 * v2.abs());
    close("c2", q[5], 1e-6 * v3, 1e-12 * 1e-6 * v3.abs());
    // KCL at node 2 (the source's resistor into the 1 kohm to ground, the 1 kohm on, and c1) and at node 3.
    // (the 1e-12 S shunt of each node over 2.6 ms and at most 3.3 V is 9e-15 C)
    close("node 2", q[1], q[2] + q[3] + q[4], 1e-9 * q[1].abs() + 2e-14);
    close("node 3", q[3], q[5], 1e-9 * q[3].abs() + 2e-14);
    let (e2, q2, bits2) = run(true);
    assert!(bits == bits2, "readings after a failed advance differ");
    assert!(e.voltages() == e2.voltages());
    for k in 0..q.len() {
        close(&format!("slot {k} after the failure"), q2[k], q[k], 1e-12 * q[k].abs().max(1e-18));
    }
}

#[test]
fn the_last_stops_length_from_1e_4_to_1e_15_s_reads_the_closed_form_on_the_ladder() {
    // CAPERR3 A3: the sub-floor rule's reach. The ladder at 50 and at 5 ohm, host stops as for
    // the sliver test, the last one `d` after the one before it, d from 1e-4 s (a plain step)
    // down to 1e-15 s: both capacitor currents at 2.33 ms against the closed form, in W0.
    let t_end = 2.33e-3;
    let mut worst = [0.0f64; 2];
    for (k, rs) in [50.0, 5.0].into_iter().enumerate() {
        let want = ladder_currents(rs, t_end);
        let mut row = String::new();
        let mut at_1e_5 = [0.0; 2];
        for d in [1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 1e-13, 1e-14, 5.86e-15, 1e-15] {
            let mut eng = Engine::load(&ladder(rs)).unwrap();
            for t in [0.05e-3, 0.13e-3, 0.31e-3, 0.52e-3, 0.77e-3, 1.21e-3, 1.55e-3, t_end - d] {
                eng.advance(t).unwrap();
            }
            eng.advance(t_end).unwrap();
            let e = [(eng.currents()[4] - want.0) / w0_i(want.0), (eng.currents()[5] - want.1) / w0_i(want.1)];
            if d == 1e-5 {
                at_1e_5 = e;
            }
            // The reading does not move with the sliver: within 0.02 W0 of the one a step of 10 us reads.
            if d < 1e-5 {
                assert!((e[0] - at_1e_5[0]).abs() <= 0.02 && (e[1] - at_1e_5[1]).abs() <= 0.02, "rs {rs}, d {d:e}: {e:?} against {at_1e_5:?}");
            }
            worst[k] = worst[k].max(e[0].abs()).max(e[1].abs());
            row += &format!(" {d:.0e}: {:+.3}/{:+.3}", e[0], e[1]);
        }
        eprintln!("rs {rs}:{row}");
    }
    assert!(worst[0] <= 1.0, "the 50 ohm ladder reads {:.2} W0 off at a last stop", worst[0]);
    assert!(worst[1] <= 2.0, "the 5 ohm ladder reads {:.2} W0 off at a last stop", worst[1]);
}

/// The readings at host stops, c1 and c2 against the ladder's closed form: the worst `(W0, at t)` over the stops.
fn ladder_at_stops(rs: f64, stops: &[f64]) -> ((f64, f64), (f64, f64)) {
    let mut eng = Engine::load(&ladder(rs)).unwrap();
    let (mut w1, mut w2) = ((0.0f64, 0.0f64), (0.0f64, 0.0f64));
    for &t in stops {
        eng.advance(t).unwrap();
        let want = ladder_currents(rs, t);
        let e1 = (eng.currents()[4] - want.0).abs() / w0_i(want.0);
        let e2 = (eng.currents()[5] - want.1).abs() / w0_i(want.1);
        if e1 > w1.0 {
            w1 = (e1, t);
        }
        if e2 > w2.0 {
            w2 = (e2, t);
        }
    }
    (w1, w2)
}

#[test]
fn part_d_the_ladders_readings_at_host_stops_for_1_5_20_and_50_ohm() {
    // CAPERR3 part D (measurement only): the app reads at host stops. 1 ms and 1/60 s stops to
    // 0.3 s, and uneven ones (a fixed pseudo-random 0.3 to 25 ms gap), the worst c1 / c2 in W0.
    let even = |h: f64| (1..).map(|k| k as f64 * h).take_while(|&t| t <= 0.3).collect::<Vec<f64>>();
    let mut uneven = vec![];
    let (mut t, mut s) = (0.0f64, 12345u64);
    while t < 0.3 {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        t += 0.3e-3 + 24.7e-3 * ((s >> 33) as f64 / (1u64 << 31) as f64);
        uneven.push(t);
    }
    for (rs, plain) in [1.0, 5.0, 20.0, 50.0].into_iter().flat_map(|r| [(r, false), (r, true)]) {
        stepped::NO_EXTRAP.with(|c| c.set(plain));
        let mut line = format!("rs {rs:>4} {}:", if plain { "plain trapezoid" } else { "extrapolated  " });
        for (name, stops) in [("1ms", even(1e-3)), ("1/60s", even(1.0 / 60.0)), ("uneven", uneven.clone())] {
            let (a, b) = ladder_at_stops(rs, &stops);
            line += &format!("  {name}: c1 {:.3} W0 @{:.4}s, c2 {:.3} W0 @{:.4}s;", a.0, a.1, b.0, b.1);
        }
        eprintln!("{line}");
    }
    stepped::NO_EXTRAP.with(|c| c.set(false));
}

// ---- CAPERR4: the extrapolated TR-BDF2 past the guard ----

/// A complex number, for the amplification factors.
#[derive(Clone, Copy)]
struct Cx(f64, f64);

impl Cx {
    fn add(self, o: Cx) -> Cx {
        Cx(self.0 + o.0, self.1 + o.1)
    }
    fn scale(self, k: f64) -> Cx {
        Cx(self.0 * k, self.1 * k)
    }
    fn mul(self, o: Cx) -> Cx {
        Cx(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn div(self, o: Cx) -> Cx {
        let d = o.0 * o.0 + o.1 * o.1;
        Cx((self.0 * o.0 + self.1 * o.1) / d, (self.1 * o.0 - self.0 * o.1) / d)
    }
    fn abs(self) -> f64 {
        self.0.hypot(self.1)
    }
}

/// TR-BDF2's amplification of `x' = -lambda x` at `z = lambda h` (the engine's
/// `gamma = 2 - sqrt(2)`): the trapezoidal stage to `gamma h`, then BDF2.
fn tr_bdf2_r(z: Cx) -> Cx {
    let g = 2.0 - std::f64::consts::SQRT_2;
    let r1 = Cx(1.0, 0.0).add(z.scale(-g / 2.0)).div(Cx(1.0, 0.0).add(z.scale(g / 2.0)));
    let num = r1.add(Cx(-(1.0 - g) * (1.0 - g), 0.0)).scale(1.0 / (g * (2.0 - g)));
    num.div(Cx(1.0, 0.0).add(z.scale((1.0 - g) / (2.0 - g))))
}

/// The extrapolated step's: `(4 R(z/2)^2 - R(z)) / 3`.
fn tr_bdf2_e(z: Cx) -> Cx {
    let half = tr_bdf2_r(z.scale(0.5));
    half.mul(half).scale(4.0).add(tr_bdf2_r(z).scale(-1.0)).scale(1.0 / 3.0)
}

#[test]
fn the_extrapolated_tr_bdf2_never_amplifies_a_real_mode_and_is_third_order() {
    // CAPERR4 item 1. On the real axis (every mode of a stepped partition, which
    // carries no inductor and whose conductance matrix is symmetric: `rate_from_matrix`)
    // `0 < E(z) < 1` for every `z > 0` from 1e-3 to 1e9 (at most 0.13 from z = 3 on), and
    // `E(z) z -> 1.61` (the extrapolated trapezoid's tends to 5/3 and crosses 1 at 25.9).
    let mut last = 1.0;
    let steps = 120_000;
    for k in 0..=steps {
        let z = 10f64.powf(-3.0 + 12.0 * k as f64 / steps as f64);
        let (r, e) = (tr_bdf2_r(Cx(z, 0.0)), tr_bdf2_e(Cx(z, 0.0)));
        assert!(r.0.abs() < 1.0 && r.1 == 0.0, "R({z:e}) = {}", r.0);
        // 1 at z -> 0, down to a minimum of 0.0526 at z = 4.03, up to a local maximum of 0.117 at z = 13.6, then 1.61 / z.
        assert!(e.0 > 0.0 && e.0 < 1.0 && (z > 3.9 || e.0 < last), "E({z:e}) = {:e} after {last:e}", e.0);
        assert!(z < 3.0 || e.0 <= 0.13, "E({z:e}) = {:e}", e.0);
        last = e.0;
    }
    assert!((tr_bdf2_e(Cx(1e9, 0.0)).0 * 1e9 - 1.6095).abs() < 1e-3, "E z at 1e9: {}", tr_bdf2_e(Cx(1e9, 0.0)).0 * 1e9);
    assert!(tr_bdf2_e(Cx(10.0, 0.0)).0 < 0.11, "E(10) = {}", tr_bdf2_e(Cx(10.0, 0.0)).0);
    // Order: halving h cuts the one-step error `|E(z) - exp(-z)|` by 2^4 (local order 4, global 3),
    // R's by 2^3.
    let err = |f: fn(Cx) -> Cx, z: f64| (f(Cx(z, 0.0)).0 - (-z).exp()).abs();
    for z in [0.05, 0.025, 0.0125] {
        let (e, r) = (err(tr_bdf2_e, z) / err(tr_bdf2_e, z / 2.0), err(tr_bdf2_r, z) / err(tr_bdf2_r, z / 2.0));
        assert!((e - 16.0).abs() < 0.5, "E's error ratio at z = {z}: {e}");
        assert!((r - 8.0).abs() < 0.3, "R's error ratio at z = {z}: {r}");
    }
}

#[test]
fn the_extrapolated_tr_bdf2_on_the_complex_plane_is_bounded_by_1_075_in_a_sliver_beside_the_imaginary_axis() {
    // TR-BDF2 is A-stable (|R| <= 1 on the closed right half plane), its extrapolation is not
    // quite: |E| reaches 1.075 at z = 1e-4 + 5.08i and exceeds 1 only within 1.4 degrees of the
    // imaginary axis (Re z < 0.13, 0.2 < |Im z| < 7.3). A partition with an inductor would have such
    // modes; a stepped partition has none (`rate_from_matrix` refuses a matrix that is not
    // symmetric and dominant, and no inductor is stamped there), and the extrapolated trapezoid's
    // own `|E|` is 5/3 there.
    let (mut worst_e, mut worst_r) = (0.0f64, 0.0f64);
    let mags: Vec<f64> = (0..=400).map(|k| 10f64.powf(-4.0 + 13.0 * k as f64 / 400.0)).collect();
    for &a in &mags {
        for &b in std::iter::once(&0.0).chain(&mags) {
            for sign in [1.0, -1.0] {
                let z = Cx(a, sign * b);
                let (e, r) = (tr_bdf2_e(z).abs(), tr_bdf2_r(z).abs());
                worst_e = worst_e.max(e);
                worst_r = worst_r.max(r);
                assert!(r <= 1.0, "|R| at {a:e} + {b:e}i is {r}");
                if a >= 0.13 {
                    assert!(e <= 1.0, "|E| at {a:e} + {b:e}i is {e}");
                }
            }
        }
    }
    assert!(worst_e <= 1.08, "|E| reaches {worst_e}");
    eprintln!("complex grid: max |R| {worst_r:.6}, max |E| {worst_e:.6}");
}

#[test]
fn a_fast_mode_at_10_to_1e5_steps_of_its_rate_after_a_discontinuity_never_grows_nor_reads_worse_than_the_plain_trapezoid() {
    // CAPERR4 item 2: as the P test above but from 10 to 1e5 steps of the fast
    // rate, the first step after the source's edge a coarse backward Euler one.
    // Past the guard the step is the extrapolated TR-BDF2: every reading within
    // W0 of the closed form or no worse than the plain trapezoid's, and no ring.
    // The fast stage is 1 ohm into `cf`, chosen so that the rate bound `2 G / cf`
    // times the 1 ms stop is `prod` (the closed form stays well conditioned,
    // which a smaller resistor would not).
    // `H_MAX` is 10 ms, so the stops are 10 ms and the fast stage's time constant
    // `cf` ohm-farads: P = 1e5 is 200 ns. A million would put it at 20 ns, two
    // floor steps, where the startup transient itself drops the partition to
    // backward Euler (R81 (1)) and no step is ever extrapolated: the amplification
    // test above covers z to 1e9.
    for prod in [10.0, 30.0, 100.0, 1e3, 1e4, 1e5] {
        let t_stop = 1e-2;
        let cf = 2.0 * t_stop / prod;
        let (p, closed) = stages(5.0, 1.0, cf, 10_000.0, 1e-6, 1e9);
        let stops: Vec<f64> = (1..=12).map(|k| k as f64 * t_stop).collect();
        stepped::NO_TRBDF2.with(|c| c.set(true));
        let (plain, _) = read_against(&p, &closed, [cf, 1e-6], [2, 4], &stops);
        stepped::NO_TRBDF2.with(|c| c.set(false));
        stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let (got, vworst) = read_against(&p, &closed, [cf, 1e-6], [2, 4], &stops);
        let (wp, ..) = worst_of(&plain);
        let (w, t, _) = worst_of(&got);
        eprintln!("P {prod:e}: plain {wp:.3} W0, TR-BDF2 {w:.3} W0 (volt {vworst:.3}), extrapolated TR-BDF2 steps {}", stepped::EXTRAP_TRBDF2.with(|c| c.get()));
        assert!(vworst <= 1.0, "P {prod}: a node reads {vworst:.2} W0 off its closed form");
        assert!(w <= wp.max(1.0) * (1.0 + 1e-9), "P {prod}: a capacitor current reads {w:.2} W0 off at t = {t:e}, the plain trapezoid {wp:.2}");
        assert!(stepped::EXTRAP_TRBDF2.with(|c| c.get()) > 0, "P {prod}: no step was past the guard");
        for j in 0..2 {
            let e: Vec<f64> = got.iter().map(|g| g.1[j]).collect();
            for k in 2..e.len() {
                let ring = e[k - 2] * e[k - 1] < 0.0 && e[k - 1] * e[k] < 0.0 && e[k - 1].abs() > 1.2 * e[k - 2].abs() && e[k].abs() > 1.2 * e[k - 1].abs() && e[k].abs() > 1.0;
                assert!(!ring, "P {prod}: capacitor {j} rings with growing amplitude at reading {k}: {:.2}, {:.2}, {:.2} W0", e[k - 2], e[k - 1], e[k]);
            }
        }
        let amp = got.iter().flat_map(|g| g.1).fold(0.0f64, |m, e| m.max(e.abs()));
        assert!(amp <= wp.max(1.0) * (1.0 + 1e-9), "P {prod}: amplitude {amp}");
    }
}

#[test]
fn a_board_whose_steps_all_pass_the_guard_is_bit_for_bit_what_it_was() {
    // CAPERR4: up to the guard nothing changes. The 500 ohm ladder's fastest rate
    // bound times its longest step is under 10: the same bits with the TR-BDF2 off.
    let stops: Vec<f64> = (1..=30).map(|k| k as f64 * 0.37e-3).collect();
    for p in [ladder(500.0), ladder(50.0), stages(5.0, 1000.0, 1e-6, 1000.0, 2e-6, 1e9).0] {
        stepped::NO_TRBDF2.with(|c| c.set(true));
        stepped::EXTRAP_TRBDF2.with(|c| c.set(0));
        let off = run_bits(&p, &stops);
        stepped::NO_TRBDF2.with(|c| c.set(false));
        let on = run_bits(&p, &stops);
        if stepped::EXTRAP_TRBDF2.with(|c| c.get()) == 0 {
            assert!(on == off, "a board with no step past the guard reads differently");
        }
    }
}


