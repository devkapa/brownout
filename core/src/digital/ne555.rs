//! The NE555 (core/README.md, "Kernel semantics: NE555 (W1e)"). Owned by W1e
//! worker E1.
//!
//! The public API below is the orchestrator's lock, declared before dispatch
//! so no worker needs to touch `digital/mod.rs`. The bodies are stubs until
//! E1 lands (the underscores go with them): nothing in the dispatcher or the
//! solver calls them yet.

use super::power::{ic_power_held, IcPower, PowerHold};
use super::IcRecord;
use crate::drivers::{Driver, DriverStage};
use crate::events::Mode;
use crate::solve::Threshold;

/// Logic indices into `IcRecord::logic` (core/README.md, "NE555 record").
pub use super::{NE555_CV as CV, NE555_DIS as DIS, NE555_OUT as OUT, NE555_RST as RST, NE555_THR as THR, NE555_TRIG as TRIG};

/// `NE555_OUTPUT_R_OHM`/`NE555_OUTPUT_HEADROOM_V` (DI:86-87): the OUT stage's
/// fixed 20 ohm output resistance and 0.1 V no-load headroom.
const OUTPUT_R_OHM: f64 = 20.0;
const OUTPUT_HEADROOM_V: f64 = 0.1;
/// The DIS stage's committed-LOW resistance, pin 7 to pin 1 (DI:216-218:
/// `stampResistor(ctx.mna, pDis, pGnd, 10)`).
const DIS_R_OHM: f64 = 10.0;
/// Point 7's "unloaded/loaded" boundary and point 8's element-current guard:
/// classic clamps `outputFraction`/tests `f > 1e-12`/`f < 1 - 1e-12` before
/// stamping either branch (`_stampRailReferencedOutput`, SE:5266-5271).
const STAMP_EPS: f64 = 1e-12;

/// The latch, the 555's only state (NE555 point 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ne555State {
    pub out_high: bool,
}

impl Ne555State {
    /// Classic's `{outHigh: true}` (DI:233).
    pub fn power_on() -> Self {
        Ne555State { out_high: true }
    }
}

/// NE555 point 3: the comparators over inputs already relative to pin 1, all
/// strict, first match wins (DI:235-244): RST beats TRIG beats THR beats the
/// held `stored` value. `v_rst` is compared as an absolute volt above pin 1
/// (0.5 V, not a supply fraction); `v_trig`/`v_thr` against `vLower`/`vUpper`,
/// each a fraction of `vs`.
pub fn resolve(stored: bool, v_rst: f64, v_trig: f64, v_thr: f64, vs: f64) -> bool {
    let v_upper = (2.0 / 3.0) * vs;
    let v_lower = (1.0 / 3.0) * vs;
    if v_rst < 0.5 {
        false
    } else if v_trig < v_lower {
        true
    } else if v_thr > v_upper {
        false
    } else {
        stored
    }
}

/// One evaluation (NE555 points 2-3 and 9): `None` while unpowered (reads,
/// writes and requests nothing, DI:180-181/:236); otherwise the point-3
/// result, written into `state` only when `acting_on_edges` (point 9.1: a
/// frozen settling pass drives the comparator result without touching the
/// latch, so a transient glitch during settling can never set or reset it).
pub fn evaluate(ic: &IcRecord, state: &mut Ne555State, voltages: &[f64], acting_on_edges: bool, hold: PowerHold) -> Option<bool> {
    let v1 = voltages[ic.gnd.node];
    // A held-off 555 is unpowered whatever its supply reads (W1c point 1a).
    let power = ic_power_held(ic, voltages[ic.vcc.node], v1, hold);
    if !power.powered {
        return None;
    }
    let v_rst = voltages[ic.logic[RST].node] - v1;
    let v_trig = voltages[ic.logic[TRIG].node] - v1;
    let v_thr = voltages[ic.logic[THR].node] - v1;
    let result = resolve(state.out_high, v_rst, v_trig, v_thr, power.vs);
    if acting_on_edges {
        state.out_high = result;
    }
    Some(result)
}

/// OUT's fraction toward pin 8 (NE555 point 7; DI:196-206): the committed
/// level's Thevenin split, `0` whenever the supply itself has collapsed to
/// `0` (unpowered is a separate, earlier gate — this only guards the
/// division by `vs`).
pub fn out_fraction(high: bool, vs: f64) -> f64 {
    if vs <= 0.0 {
        return 0.0;
    }
    let raw = if high { 1.0 - (OUTPUT_HEADROOM_V / vs).min(0.5) } else { (OUTPUT_HEADROOM_V / vs).min(0.5) };
    raw.clamp(0.0, 1.0)
}

/// The `(target node, conductance)` branches a 555 driver stamps from its
/// pin (NE555 point 7): OUT can stamp up to two (pin 3 to pin 8 at `f/20` S,
/// pin 3 to pin 1 at `(1-f)/20` S — classic's complementary-conductance
/// Thevenin split, so an unloaded pin still draws the idle cross-current
/// between the rails); DIS stamps at most one (pin 7 to pin 1 at `1/10` S
/// while committed LOW; released, stamping nothing, while committed HIGH).
/// `committed` is `driver.committed()`, threaded through explicitly (the
/// orchestrator's locked signature) rather than read off `driver` here, so
/// this stays a plain function of values, like every other export in this
/// module. `power`/`ic` are already known powered by the only caller
/// (`solve::ic_driver_branches`), which gates on `power.powered` before ever
/// reaching here.
pub fn branches(stage: DriverStage, committed: Option<Mode>, power: &IcPower, _driver: &Driver, ic: &IcRecord) -> [Option<(usize, f64)>; 2] {
    match stage {
        DriverStage::Ne555Out => {
            let high = match committed {
                Some(Mode::High) => true,
                Some(Mode::Low) => false,
                // Uncommitted or mid-HIZ: OUT never actually requests HIZ
                // (point 4 only ever requests HIGH/LOW), but a driver that
                // has never committed at all stamps nothing, same as any
                // other IC output (the generic `Logic` stage's identical
                // `_ => None` fallback).
                _ => return [None, None],
            };
            let f = out_fraction(high, power.vs);
            let mut out = [None, None];
            if f > STAMP_EPS {
                out[0] = Some((ic.vcc.node, f / OUTPUT_R_OHM));
            }
            if f < 1.0 - STAMP_EPS {
                out[1] = Some((ic.gnd.node, (1.0 - f) / OUTPUT_R_OHM));
            }
            out
        }
        DriverStage::Ne555Dis => match committed {
            Some(Mode::Low) => [Some((ic.gnd.node, 1.0 / DIS_R_OHM)), None],
            // Committed HIGH: released, stamps nothing (DI:216: `!outHigh &&
            // pDis >= 0`). Uncommitted/HIZ: likewise nothing.
            _ => [None, None],
        },
        // Unreachable through `solve::ic_driver_branches` (it only ever
        // dispatches a 555's own two stages here), but total rather than
        // `unreachable!()`: this module promises pure functions with no
        // panics on a value that type-checks.
        DriverStage::Logic => [None, None],
    }
}

/// The conductances [`branches`] stamps, as curves in the supply difference
/// (in the same slots, whether or not `STAMP_EPS` stamps them): OUT's
/// `f / 20` to pin 8 and `(1 - f) / 20` to pin 1, `f` the headroom split
/// [`out_fraction`] clamps at `vs = 2 * headroom`; DIS's constant `1 / 10`.
pub fn branch_curves(stage: DriverStage, committed: Option<Mode>) -> [Option<super::power::SupplyCurve>; 2] {
    use super::power::SupplyCurve;
    let (h, r) = (OUTPUT_HEADROOM_V, OUTPUT_R_OHM);
    // `H / vs` over `r`, and its complement, each held at `1 / 2r` below the
    // clamp.
    let split = |beta_sign: f64| SupplyCurve {
        kinks: [2.0 * h, f64::INFINITY],
        pieces: [(0.5 / r, 0.0), (if beta_sign < 0.0 { 1.0 / r } else { 0.0 }, beta_sign * h / r), (if beta_sign < 0.0 { 1.0 / r } else { 0.0 }, beta_sign * h / r)],
    };
    match (stage, committed) {
        (DriverStage::Ne555Out, Some(Mode::High)) => [Some(split(-1.0)), Some(split(1.0))],
        (DriverStage::Ne555Out, Some(Mode::Low)) => [Some(split(1.0)), Some(split(-1.0))],
        (DriverStage::Ne555Dis, Some(Mode::Low)) => [Some(SupplyCurve::constant(1.0 / DIS_R_OHM)), None],
        _ => [None, None],
    }
}

/// The current delivered out of pin 3 (NE555 point 8; DI:252-273): the same
/// Thevenin split as [`branches`], but reported unconditionally (classic
/// publishes the formula even when nothing was actually stamped, node 0
/// included — E, question 4) rather than gated on the `STAMP_EPS` guard that
/// decides whether a branch is worth stamping at all.
pub fn element_current(high: bool, vs: f64, v_out: f64, v_gnd: f64, v_vcc: f64) -> f64 {
    let f = out_fraction(high, vs);
    let g_hi = f / OUTPUT_R_OHM;
    let g_lo = (1.0 - f) / OUTPUT_R_OHM;
    -(g_lo * (v_out - v_gnd) + g_hi * (v_out - v_vcc))
}

/// The crossing thresholds for one read pin, both directions (NE555 point
/// 5): pin 2 (TRIG) at `V(1) + vLower`, pin 6 (THR) at `V(1) + vUpper`, pin 4
/// (RST) at `V(1) + 0.5`; empty for any other logic index (OUT/DIS/CV are
/// never watched). `v_vcc`/`v_gnd` are read fresh by the caller from the
/// live, published array at every re-aim (this function does no lookups of
/// its own), so a threshold always follows the live supply, whatever the
/// power state (point 5: "whatever the power state").
pub fn watch_thresholds(logic_index: usize, v_vcc: f64, v_gnd: f64) -> Vec<Threshold> {
    let vs = (v_vcc - v_gnd).max(0.0);
    let value = match logic_index {
        TRIG => v_gnd + (1.0 / 3.0) * vs,
        THR => v_gnd + (2.0 / 3.0) * vs,
        RST => v_gnd + 0.5,
        _ => return Vec::new(),
    };
    vec![Threshold { value, rising: true }, Threshold { value, rising: false }]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digital::power::ic_power;
    use crate::digital::IcPin;
    use crate::engine::Engine;
    use crate::kinds::{self, ic_param};
    use crate::partition::{self, PartitionKind};
    use crate::program::{Device, Options, PinRecord, Program, PIN_FLAG_OPEN};

    // ======================================================================
    // Pure-function tests (draft section D, E1 items 1-2). No `Engine`
    // needed: `resolve`/`out_fraction`/`element_current` are plain functions
    // of already-relative voltages.
    // ======================================================================

    // ---- item 1: resolve's truth table -----------------------------------

    #[test]
    fn resolve_rst_wins_over_trig_and_thr() {
        // vTrig well below vLower (would SET) and vThr well above vUpper
        // (would RESET) simultaneously -- RST still wins (point 3, branch 1).
        assert!(!resolve(true, 0.4, 0.0, 10.0, 5.0));
        assert!(!resolve(false, 0.0, 0.0, 10.0, 5.0));
    }

    #[test]
    fn resolve_set_beats_reset_when_both_comparators_are_true() {
        // vTrig < vLower (1.667) AND vThr > vUpper (3.333): SET must win
        // (DI:235-244, the monostable-hold tie-break), whatever `stored` is.
        assert!(resolve(false, 5.0, 0.0, 10.0, 5.0));
        assert!(resolve(true, 5.0, 0.0, 10.0, 5.0));
    }

    #[test]
    fn resolve_thr_resets_when_trig_alone_does_not_set() {
        assert!(!resolve(true, 5.0, 5.0, 10.0, 5.0));
        assert!(!resolve(false, 5.0, 5.0, 10.0, 5.0));
    }

    #[test]
    fn resolve_holds_the_stored_value_between_the_thresholds() {
        assert!(resolve(true, 5.0, 2.5, 2.5, 5.0));
        assert!(!resolve(false, 5.0, 2.5, 2.5, 5.0));
    }

    #[test]
    fn resolve_is_strict_a_node_exactly_on_a_threshold_changes_nothing() {
        // Written the way classic writes them, `(1/3) * vs`, so the node lands
        // on the threshold bit for bit (`5.0 / 3.0` differs in the last ulp).
        let v_lower = (1.0 / 3.0) * 5.0;
        let v_upper = (2.0 / 3.0) * 5.0;
        assert!(resolve(true, 5.0, v_lower, v_upper, 5.0), "exactly AT vLower/vUpper: neither comparator is strictly true, so the stored HIGH holds");
        assert!(!resolve(false, 5.0, v_lower, v_upper, 5.0), "...and the stored LOW holds too");
        assert!(resolve(true, 0.5, v_lower, 0.0, 5.0), "vRst exactly 0.5 is NOT < 0.5, so it does not reset");
    }

    #[test]
    fn evaluate_thresholds_follow_pin1_not_an_absolute_ground() {
        // Every comparator input is V(pin) - V(1) (NE555 point 2), never an
        // absolute node voltage: shifting pin 1 (and every other node with
        // it) by a constant must not change the comparator result.
        let ic = test_ic();
        let mut state_a = Ne555State { out_high: false };
        // vS = 5, vLower = 1.667 (relative to pin 1 = 0V here). TRIG at 1.5V
        // is below vLower: SET.
        let voltages_a = [0.0, 5.0, 1.5, 0.0, 5.0, 0.0, 0.0, 0.0];
        let out_a = evaluate(&ic, &mut state_a, &voltages_a, true, PowerHold::FREE).unwrap();
        assert!(out_a, "TRIG 1.5V < vLower 1.667V (pin1=0V): SET");

        let mut state_b = Ne555State { out_high: false };
        // Pin 1 (gnd) shifted to +1V, and every node with it: every voltage
        // RELATIVE to pin 1 is identical, so the verdict must be identical.
        let voltages_b = [1.0, 6.0, 2.5, 0.0, 6.0, 0.0, 1.0, 0.0];
        let out_b = evaluate(&ic, &mut state_b, &voltages_b, true, PowerHold::FREE).unwrap();
        assert_eq!(out_a, out_b, "shifting pin 1 (and every node with it) by +1V must not change the comparator result");
    }

    #[test]
    fn evaluate_ignores_cv_entirely() {
        let ic = test_ic();
        let mut state_a = Ne555State { out_high: true };
        let mut state_b = Ne555State { out_high: true };
        let mut va = [0.0, 5.0, 5.0, 0.0, 5.0, 0.0, 5.0, 0.0];
        let mut vb = va;
        va[CV] = 0.0;
        vb[CV] = 123.456; // CV (pin 5) at an arbitrary voltage.
        let out_a = evaluate(&ic, &mut state_a, &va, true, PowerHold::FREE);
        let out_b = evaluate(&ic, &mut state_b, &vb, true, PowerHold::FREE);
        assert_eq!(out_a, out_b, "CV is never read (NE555 point 2)");
    }

    #[test]
    fn evaluate_none_while_unpowered_and_holds_the_latch_untouched() {
        let ic = test_ic();
        let mut state = Ne555State { out_high: true };
        // vS well under the 2.0*0.9 = 1.8V floor: unpowered.
        let voltages = [0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        assert_eq!(evaluate(&ic, &mut state, &voltages, true, PowerHold::FREE), None);
        assert!(state.out_high, "an unpowered evaluation reads, writes and requests nothing: the latch is untouched");
    }

    #[test]
    fn evaluate_a_frozen_pass_drives_the_result_without_writing_the_latch() {
        // Point 9.1: TRIG well below vLower would SET, but with
        // acting_on_edges = false the RESULT is still the live comparator
        // verdict (what a frozen settling pass requests OUT/DIS from) -- only
        // the STORED latch must stay untouched.
        let ic = test_ic();
        let mut state = Ne555State { out_high: false };
        let voltages = [0.0, 5.0, 0.0, 5.0, 5.0, 0.0, 0.0, 0.0];
        let result = evaluate(&ic, &mut state, &voltages, false, PowerHold::FREE).unwrap();
        assert!(result, "the frozen pass's own requested result reflects the live comparators");
        assert!(!state.out_high, "...but the stored latch must not have been written");
    }

    // ---- item 2: the output stage, loaded and unloaded (the probe's own
    // values) --------------------------------------------------------------

    #[test]
    fn output_stage_matches_the_probes_values_loaded_and_unloaded() {
        let vs = 5.0;
        let load_g = 1.0 / 1000.0;

        // HIGH into 1k: V(out) = 4.9*1000/1020, elementI = 4.803921568627451e-3.
        let f_hi = out_fraction(true, vs);
        assert!((f_hi - 0.98).abs() < 1e-15, "f_hi={f_hi}");
        let g_hi_vcc = f_hi / OUTPUT_R_OHM;
        let g_hi_gnd = (1.0 - f_hi) / OUTPUT_R_OHM;
        let v_hi = (g_hi_vcc * vs) / (g_hi_vcc + g_hi_gnd + load_g);
        assert!((v_hi - 4.803_921_568_627_451).abs() < 1e-12, "v_hi={v_hi}");
        let i_hi = element_current(true, vs, v_hi, 0.0, vs);
        assert!((i_hi - 4.803_921_568_627_451e-3).abs() < 1e-15, "i_hi={i_hi}");

        // LOW into 1k: V(out) = 0.09803921568627451, elementI = 9.803921568627451e-5.
        let f_lo = out_fraction(false, vs);
        assert!((f_lo - 0.02).abs() < 1e-15, "f_lo={f_lo}");
        let g_lo_vcc = f_lo / OUTPUT_R_OHM;
        let g_lo_gnd = (1.0 - f_lo) / OUTPUT_R_OHM;
        let v_lo = (g_lo_vcc * vs) / (g_lo_vcc + g_lo_gnd + load_g);
        assert!((v_lo - 0.098_039_215_686_274_51).abs() < 1e-12, "v_lo={v_lo}");
        let i_lo = element_current(false, vs, v_lo, 0.0, vs);
        assert!((i_lo - 9.803_921_568_627_45e-5).abs() < 1e-15, "i_lo={i_lo}");

        // No load: 4.9V/0.1V, with the source current -4.9e-3 A in both
        // states (the idle cross-current, DI:197-202).
        let v_hi_unloaded = (g_hi_vcc * vs) / (g_hi_vcc + g_hi_gnd);
        assert!((v_hi_unloaded - 4.9).abs() < 1e-12);
        let v_lo_unloaded = (g_lo_vcc * vs) / (g_lo_vcc + g_lo_gnd);
        assert!((v_lo_unloaded - 0.1).abs() < 1e-12);
        assert!((g_hi_vcc * (vs - v_hi_unloaded) - 4.9e-3).abs() < 1e-12);
        assert!((g_lo_vcc * (vs - v_lo_unloaded) - 4.9e-3).abs() < 1e-12);
    }

    #[test]
    fn the_branch_curves_are_the_conductances_branches_stamps() {
        // The curvature band reads the OUT and DIS conductances' derivatives
        // off these curves: slot for slot they are what `branches` stamps,
        // across the headroom clamp at 2 * 0.1 V.
        let ic = test_ic();
        let drv = Driver::new_ic_output(3, 1, 0, 0, false);
        for stage in [DriverStage::Ne555Out, DriverStage::Ne555Dis] {
            for committed in [Some(Mode::High), Some(Mode::Low), None] {
                let curves = branch_curves(stage, committed);
                for k in 1..=2000 {
                    let vs = 0.01 * k as f64;
                    let power = ic_power(&ic, vs, 0.0);
                    for (slot, (stamped, curve)) in branches(stage, committed, &power, &drv, &ic).into_iter().zip(curves).enumerate() {
                        match (stamped, curve) {
                            (Some((_, g)), Some(c)) => assert!((c.derivs(vs)[0] - g).abs() <= 1e-15, "{stage:?} {committed:?} slot {slot} at {vs}: {} against {g}", c.derivs(vs)[0]),
                            (None, _) => {}
                            (Some(_), None) => panic!("{stage:?} {committed:?} slot {slot} at {vs}: stamped with no curve"),
                        }
                    }
                }
            }
        }
    }

    /// A bare `IcRecord` for the pure `evaluate`/`resolve` tests above: pins
    /// in the NE555 record's own order (`1`=gnd=node 0, `8`=vcc=node 1, then
    /// `2 3 4 5 6 7` = TRIG OUT RST CV THR DIS = nodes 2..7), no catalog
    /// window (2.0-18.0 V).
    fn test_ic() -> IcRecord {
        let p = |node: usize| IcPin { node, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false };
        IcRecord {
            kind: kinds::NE555,
            vcc: p(1),
            gnd: p(0),
            logic: vec![p(2), p(3), p(4), p(5), p(6), p(7)],
            vcc_min: 2.0,
            vcc_max: 18.0,
            ttl: false,
            cmos_ratio: false,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 5.0,
            io_max: None,
            delay: 0.0,
            slot_base: 0,
            delays: Vec::new(),
            extra: Vec::new(),
        }
    }

    // ======================================================================
    // Engine tests (draft section D, E1 items 3-13): hand-built `Program`s
    // through the real `Engine`, no TS compiler involved.
    // ======================================================================

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn open_pin(node: usize) -> PinRecord {
        PinRecord { node, flags: PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }
    }

    /// The 13 IC params for a hand-built `ne555` device: no catalog window
    /// (2.0-18.0 V, matching the draft's fixtures, "no catalogUid"); params
    /// 7-9/11/12 stay at their zero default (the reader's own rule).
    fn ne555_params() -> Vec<f64> {
        let mut p = vec![0.0; ic_param::COUNT];
        p[ic_param::VCC_MIN] = 2.0;
        p[ic_param::VCC_MAX] = 18.0;
        p[ic_param::NOMINAL] = 5.0;
        p
    }

    /// An `ne555` device: pins in the record's own order (`8 1 2 3 4 5 6 7`),
    /// `logic` already in TRIG/OUT/RST/CV/THR/DIS order.
    fn ne555_dev(ci: usize, vcc: usize, gnd: usize, logic: [PinRecord; 6], slot_base: usize) -> Device {
        let mut pins = vec![pin(vcc), pin(gnd)];
        pins.extend(logic);
        Device { kind: kinds::NE555, component_index: ci, pins, params: ne555_params(), blob: vec![slot_base as f64, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }
    }

    fn source(ci: usize, pos: usize, neg: usize, v: f64) -> Device {
        Device { kind: kinds::VOLTAGE_SOURCE, component_index: ci, pins: vec![pin(pos), pin(neg)], params: vec![v], blob: vec![] }
    }

    fn resistor(ci: usize, a: usize, b: usize, r: f64) -> Device {
        Device { kind: kinds::RESISTOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![r], blob: vec![] }
    }

    fn capacitor(ci: usize, a: usize, b: usize, c: f64) -> Device {
        Device { kind: kinds::CAPACITOR, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![c, 0.0, 0.0], blob: vec![] }
    }

    fn switch_dev(ci: usize, a: usize, b: usize, closed: f64) -> Device {
        Device { kind: kinds::SWITCH, component_index: ci, pins: vec![pin(a), pin(b)], params: vec![closed], blob: vec![] }
    }

    fn program(node_count: usize, digital_slot_count: usize, devices: Vec<Device>) -> Program {
        Program { node_count, options: Options::default(), digital_slot_count, channel_slot_count: 0, devices }
    }

    /// Flat `(t, value)` pairs for every recorded change on digital slot 0,
    /// in trace order.
    fn slot0_trace(engine: &Engine) -> Vec<(f64, f64)> {
        engine.trace().chunks_exact(3).filter(|c| c[1] as usize == 0).map(|c| (c[0], c[2])).collect()
    }

    // ---- item 3: KCL on the idle-draw benchmark circuit -------------------

    #[test]
    fn kcl_idle_draw_benchmark_matches_the_physics_reference() {
        // 0=gnd, 1=vcc(5V fixed), 2=out(1k load), 3=dis(isolated).
        // TRIG on GND (SETs), RST on VCC (never resets), THR/CV on GND.
        let p = program(4, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 2, 0, 1000.0),
            ne555_dev(2, 1, 0, [pin(0), pin(2), pin(1), pin(0), pin(0), pin(3)], 0),
        ]);
        let engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "TRIG on GND sets OUT HIGH");

        let v_out = engine.voltages()[2];
        let expected_v_out = 4.803_921_568_627_451;
        assert!((v_out - expected_v_out).abs() < 1e-9, "V(out)={v_out}");

        let expected_source_i = -(5.0 - expected_v_out) * 0.98 / 20.0;
        assert!((expected_source_i - (-9.607_843_137e-3)).abs() < 1e-9, "sanity check against the draft's own stated value");
        let source_i = engine.currents()[0];
        assert!((source_i - expected_source_i).abs() < 1e-9, "source I={source_i}, want {expected_source_i}");

        let load_i = v_out / 1000.0;
        let ne555_i = engine.currents()[2];
        // The OUT node also carries its 1e-12 S shunt (W0 point 2), so the
        // element current exceeds the resistor's by V(out) * 1e-12.
        assert!((ne555_i - load_i).abs() < 1e-10, "elementI(timer)={ne555_i} must equal the load current {load_i}");
    }

    // ---- item 4: closed-form astable against the textbook sidecar formula -

    /// RA=RB=1k, C=1uF, ideal 5V supply, OUT into 1k -- a plain textbook
    /// astable (not the corpus's F-A, which carries an extra ~1 mOhm trim pot
    /// this crate's own tests have no business reproducing exactly); every
    /// expected number below is the README point 13.3 sidecar formula,
    /// shunts dropped (negligible at this R/C scale: 1e-12 S against ~1e-3 S
    /// stamped conductances is ~1e-9 relative, the README's own "at most
    /// 3e-8 relative" for this magnitude of fixture).
    fn astable_program(ra: f64, rb: f64, c: f64, force_stepped: bool) -> Program {
        // 0=gnd, 1=vcc, 2=dis, 3=cap(=TRIG=THR), 4=out.
        let mut p = program(5, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 2, ra),
            resistor(2, 2, 3, rb),
            capacitor(3, 3, 0, c),
            resistor(4, 4, 0, 1000.0),
            ne555_dev(5, 1, 0, [pin(3), pin(4), pin(1), pin(0), pin(3), pin(2)], 0),
        ]);
        p.options.force_stepped = force_stepped;
        p
    }

    #[test]
    fn closed_form_astable_period_matches_the_textbook_sidecar_formula() {
        let (ra, rb, c, vs) = (1000.0_f64, 1000.0_f64, 1e-6_f64, 5.0_f64);
        let p = astable_program(ra, rb, c, false);

        let pb = partition::build(&p).unwrap();
        let (timing_partition, _) = pb.local_index[3].expect("the cap node is free");
        assert_eq!(pb.kinds[timing_partition], PartitionKind::ClosedForm, "an ideal-supply astable's timing partition is closed-form (NE555 point 6)");

        let mut engine = Engine::load(&p).unwrap();
        engine.trace_enable(true);
        engine.advance(0.02).unwrap();

        let v_upper = 2.0 * vs / 3.0;
        let rc = ra + rb;
        let first_high = rc * c * (vs / (vs - v_upper)).ln();
        let t_high = rc * c * 2.0_f64.ln();
        let k = 10.0 / (ra + 10.0);
        let rd = rb + ra * 10.0 / (ra + 10.0);
        let t_low = rd * c * ((2.0 - 3.0 * k) / (1.0 - 3.0 * k)).ln();
        let period = t_high + t_low;

        let trace = slot0_trace(&engine);
        let falls: Vec<f64> = trace.iter().filter(|&&(_, v)| v == 0.0).map(|&(t, _)| t).collect();
        let rises: Vec<f64> = trace.iter().filter(|&&(_, v)| v == 1.0).map(|&(t, _)| t).collect();
        assert!(falls.len() >= 2 && rises.len() >= 2, "falls={falls:?} rises={rises:?}");

        let rel = |a: f64, b: f64| ((a - b) / b).abs();
        assert!(rel(falls[0], first_high) < 1e-8, "first HIGH: got {}, want {first_high}", falls[0]);
        assert!(rel(rises[0] - falls[0], t_low) < 1e-8, "t_low (first discharge, from vUpper): got {}, want {t_low}", rises[0] - falls[0]);
        assert!(rel(falls[1] - rises[0], t_high) < 1e-8, "t_high (second charge, from vLower): got {}, want {t_high}", falls[1] - rises[0]);
        assert!(rel(rises[1] - rises[0], period) < 1e-8, "T = r_2 - r_1: got {}, want {period}", rises[1] - rises[0]);

        // Stat 9: exactly 4 crossings per period (two switches, two echoes).
        let mut probe = Engine::load(&p).unwrap();
        probe.advance(rises[0] + 1e-7).unwrap();
        let c1 = probe.stat(9);
        probe.advance(rises[1] + 1e-7).unwrap();
        let c2 = probe.stat(9);
        assert_eq!(c2 - c1, 4.0, "stat 9 must grow by exactly 4 across one full period");
    }

    // ---- item 5: the RST watch releases reset at the RC crossing ----------

    #[test]
    fn rst_watch_releases_reset_at_the_rc_crossing_time() {
        // F-A's astable plus RST on 100k to VCC with 1uF to GND (F-G).
        // 0=gnd,1=vcc,2=dis,3=cap,4=out,5=rst node.
        let p = {
            let mut p = program(6, 1, vec![
                source(0, 1, 0, 5.0),
                resistor(1, 1, 2, 1000.0),
                resistor(2, 2, 3, 1000.0),
                capacitor(3, 3, 0, 1e-6),
                resistor(4, 4, 0, 1000.0),
                resistor(5, 1, 5, 100_000.0),
                capacitor(6, 5, 0, 1e-6),
                ne555_dev(7, 1, 0, [pin(3), pin(4), pin(5), pin(0), pin(3), pin(2)], 0),
            ]);
            p.digital_slot_count = 1;
            p
        };
        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0, "RST at 0V (< 0.5V) resets OUT LOW at load");

        engine.trace_enable(true);
        engine.advance(0.02).unwrap();
        let rises: Vec<f64> = slot0_trace(&engine).into_iter().filter(|&(_, v)| v == 1.0).map(|(t, _)| t).collect();
        assert!(!rises.is_empty(), "RST must eventually release");

        let release = 100_000.0 * 1e-6 * (5.0_f64 / (5.0 - 0.5)).ln();
        assert!(((rises[0] - release) / release).abs() < 1e-8, "release: got {}, want {release}", rises[0]);
    }

    // ---- item 6: the watch invariant (both directions matter) -------------

    #[test]
    fn watch_invariant_thr_falling_prevents_a_stale_reset_after_trig_releases() {
        // 0=gnd,1=vcc,2=trig,3=thr,4=out,5=dis(isolated),6=discharge node.
        // TRIG has its own RC (10k, 1uF) so its release is a closed-form
        // crossing INSIDE one advance, an event unrelated to THR's partition:
        // the 555 is evaluated there and reads THR's PUBLISHED voltage.
        let p = program(7, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 2, 10_000.0), // TRIG charge path from VCC.
            switch_dev(2, 2, 0, 1.0),    // TRIG held LOW (closed).
            resistor(3, 1, 3, 10_000.0), // THR charge path from VCC.
            capacitor(4, 3, 0, 1e-6),    // THR's own capacitor.
            switch_dev(5, 3, 6, 0.0),    // THR discharge switch, initially open.
            resistor(6, 6, 0, 1000.0),   // ...through 1k to GND.
            resistor(7, 4, 0, 1000.0),   // OUT's load (not load-bearing here).
            ne555_dev(8, 1, 0, [pin(2), pin(4), pin(1), pin(0), pin(3), pin(5)], 0),
            capacitor(9, 2, 0, 1e-6),    // TRIG's own capacitor.
        ]);
        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "TRIG held low sets OUT HIGH at load");

        // THR charges well past vUpper (10k*1uF = 10ms) while TRIG stays
        // held low: SET must keep winning over RESET (NE555 point 3).
        engine.advance(0.05).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "SET beats RESET while TRIG is still held low");
        let v_upper = 2.0 * 5.0 / 3.0;
        assert!(engine.voltages()[3] > v_upper, "THR must actually have crossed vUpper by now");

        // One batch: start THR's discharge (1k dominates its 10k charge path;
        // it falls through vUpper after ~0.4 ms) AND release TRIG (it charges
        // through vLower after ~4 ms). The two crossings sit in different
        // partitions with no horizon clamp between them. When TRIG crosses
        // and the 555 evaluates, THR's published voltage is only current if
        // THR's own falling crossing was watched: without it the 555 reads
        // the value published at the batch (above vUpper) and RESETS.
        engine.trace_enable(true);
        engine.set_params(&[(5, 0, 1.0), (2, 0, 0.0)]).unwrap();
        engine.advance(engine.time() + 0.02).unwrap();
        assert!(engine.voltages()[3] < v_upper, "THR must have fallen back below vUpper");
        assert!(engine.voltages()[2] > 5.0 / 3.0, "TRIG must have charged past vLower");
        assert_eq!(engine.digital_slots()[0], 1.0, "OUT must stay HIGH: THR's watch kept its published voltage in sync");
        assert!(slot0_trace(&engine).iter().all(|&(_, v)| v != 0.0), "OUT must never have dipped LOW: {:?}", slot0_trace(&engine));
    }

    // ---- item 7: load settling / the phantom-edge divergence --------------

    fn hc74_params() -> Vec<f64> {
        let mut p = vec![0.0; ic_param::COUNT];
        p[ic_param::VCC_MIN] = 2.0;
        p[ic_param::VCC_MAX] = 6.0;
        p[ic_param::NOMINAL] = 5.0;
        p.extend([20e-9, 20e-9, 20e-9, 20e-9]); // clkQ, preClrQ pairs.
        p
    }

    #[allow(clippy::too_many_arguments)]
    fn hc74_dev(ci: usize, vcc: usize, gnd: usize, clr1_n: usize, d1: usize, clk1: usize, pre1_n: usize, q1: usize, q1_n: usize, q2_n: usize, q2: usize, pre2_n: usize, clk2: usize, d2: usize, clr2_n: usize, slot_base: usize) -> Device {
        let pins = vec![pin(vcc), pin(gnd), pin(clr1_n), pin(d1), pin(clk1), pin(pre1_n), pin(q1), pin(q1_n), pin(q2_n), pin(q2), pin(pre2_n), pin(clk2), pin(d2), pin(clr2_n)];
        let mut blob = vec![slot_base as f64];
        blob.extend([0.0; 12]);
        Device { kind: kinds::IC_74HC74, component_index: ci, pins, params: hc74_params(), blob }
    }

    #[test]
    fn load_settling_keeps_the_phantom_edge_for_an_astable_but_not_for_rst_held_low() {
        // 0=gnd,1=vcc,2=out/clk1,3=q1,4=q1_n,5=dis(isolated),6=q2_n(unused),7=q2(unused).
        // Digital slots: 0 = the 555, 1..4 = q1 q1_n q2 q2_n.
        let build = |rst_low: bool| -> Program {
            let (trig, rst, thr) = if rst_low {
                (0, 0, 0) // RST=GND forces reset every evaluation: OUT LOW.
            } else {
                (0, 1, 0) // TRIG=GND sets; RST=VCC never resets: OUT HIGH.
            };
            program(8, 5, vec![
                source(0, 1, 0, 5.0),
                ne555_dev(1, 1, 0, [pin(trig), pin(2), pin(rst), pin(0), pin(thr), pin(5)], 0),
                hc74_dev(2, 1, 0, /*clr1_n*/ 1, /*d1*/ 4, /*clk1*/ 2, /*pre1_n*/ 1, /*q1*/ 3, /*q1_n*/ 4, /*q2_n*/ 6, /*q2*/ 7, /*pre2_n*/ 1, /*clk2*/ 0, /*d2*/ 0, /*clr2_n*/ 1, 1),
            ])
        };

        // A sequential IC's slot is the evaluator's own view, published at
        // the instant of the edge pass (sequential point 8), so the driver's
        // own commit is read off q1's node voltage instead: the 20 ns clkQ.
        let held_low = Engine::load(&build(true)).unwrap();
        assert_eq!(held_low.digital_slots()[0], 0.0, "RST held low: OUT LOW at START_TIME");
        assert_eq!(held_low.digital_slots()[1], 0.0, "q1 keeps its power-on 0: no phantom edge");
        assert!(held_low.voltages()[3] < 1.0, "q1 pin LOW: {}", held_low.voltages()[3]);

        let mut astable = Engine::load(&build(false)).unwrap();
        assert_eq!(astable.digital_slots()[0], 1.0, "OUT HIGH at START_TIME");
        assert!(astable.voltages()[3] < 1.0, "q1 pin still LOW at START_TIME, before clkQ: {}", astable.voltages()[3]);
        astable.advance(10e-9).unwrap();
        assert!(astable.voltages()[3] < 1.0, "q1 pin still LOW at START_TIME + 10 ns");
        astable.advance(30e-9).unwrap();
        assert!(astable.voltages()[3] > 4.0, "the phantom rising edge (TRIG below vLower at load resolves HIGH) is kept: q1 pin HIGH after clkQ = 20 ns: {}", astable.voltages()[3]);
        assert_eq!(astable.digital_slots()[1], 1.0);

        // The held-low engine never sees an edge, however long it runs.
        let mut held_low = held_low;
        held_low.advance(1e-3).unwrap();
        assert_eq!(held_low.digital_slots()[1], 0.0, "still no phantom edge: OUT never rose");
        assert!(held_low.voltages()[3] < 1.0);
    }

    // ---- item 8: Q-O16, an open RST ----------------------------------------

    #[test]
    fn open_rst_gives_out_low_from_start_time_with_no_crossing() {
        // 0=gnd,1=vcc,2=out(1k load),3=rst(isolated, OPEN),4=dis(isolated).
        let p = program(5, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 2, 0, 1000.0),
            ne555_dev(2, 1, 0, [pin(0), pin(2), open_pin(3), pin(0), pin(0), pin(4)], 0),
        ]);
        let engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0, "an unwired RST reads 0V through the shunt: reset, OUT LOW");
        let v_out = engine.voltages()[2];
        assert!((v_out - 0.098_039_215_686_274_51).abs() < 1e-9, "V(out)={v_out}, the LOW-loaded value -- no one-solve HIGH frame");
        assert_eq!(engine.stat(9), 0.0, "an isolated RST node never crosses anything");
    }

    // ---- item 9: power loss holds the latch, does not recompute it --------

    #[test]
    fn power_loss_freezes_the_latch_and_restores_it_held_not_recomputed() {
        // An astable whose VCC (pin 8) comes through a switch from the 5V
        // source: 0=gnd, 1=source+ (also the timing network's own rail),
        // 2=555 vcc (free, with a 1k bleeder so an opened switch collapses it
        // decisively rather than leaving a marginal supply for the power-gate
        // fixed point to limit-cycle on), 3=dis, 4=cap (=TRIG=THR), 5=out.
        // DIS and OUT are free output pins, so they join the free supply's
        // partition, and RB/C make it dynamic: stepped (NE555 point 6).
        let (ra, rb, c) = (1000.0_f64, 1000.0_f64, 1e-6_f64);
        let p = program(6, 1, vec![
            source(0, 1, 0, 5.0),
            switch_dev(1, 1, 2, 1.0),
            resistor(2, 2, 0, 1000.0),
            resistor(3, 1, 3, ra),
            resistor(4, 3, 4, rb),
            capacitor(5, 4, 0, c),
            resistor(6, 5, 0, 1000.0),
            ne555_dev(7, 2, 0, [pin(4), pin(5), pin(1), pin(0), pin(4), pin(3)], 0),
        ]);
        let pb = partition::build(&p).unwrap();
        let (p_idx, _) = pb.local_index[2].expect("the 555's own vcc pin is a free node");
        assert_eq!(pb.kinds[p_idx], PartitionKind::Stepped, "a dynamic partition holding a free IC supply is stepped (NE555 point 6)");
        assert_eq!(pb.local_index[4].map(|(q, _)| q), Some(p_idx), "the timing network shares that partition");

        let mut engine = Engine::load(&p).unwrap();
        engine.trace_enable(true);
        // Mid the first HIGH (which runs to ~2.2 ms): cut the supply.
        engine.advance(1e-3).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "mid-HIGH");
        engine.set_params(&[(1, 0, 0.0)]).unwrap();
        engine.advance(engine.time() + 0.3e-3).unwrap();
        assert!(engine.digital_slots()[0].is_nan(), "unpowered: the slot goes NaN");
        assert_eq!(engine.currents()[7], 0.0, "unpowered: elementI is 0");
        assert!(engine.voltages()[5].abs() < 1e-6, "OUT stamps nothing while unpowered: its node settles to 0V through its own load alone, got {}", engine.voltages()[5]);

        // Restore. The timing capacitor kept charging from the rail, and
        // still sits between the thresholds, so the comparators hold: the
        // latch is HIGH only if the hold survived the outage.
        let v_held = engine.voltages()[4];
        assert!(v_held > 5.0 / 3.0 && v_held < 10.0 / 3.0, "the capacitor must still sit between vLower and vUpper: {v_held}");
        engine.set_params(&[(1, 0, 1.0)]).unwrap();
        let t_restore = engine.time();
        engine.advance(t_restore + 0.3e-3).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "the latch comes back HIGH: HELD through the outage, not reset to some other default");

        // It falls when the capacitor next reaches vUpper: Rc C ln((vS - v0) /
        // (vS - vUpper)) after the restore, from the voltage v0 read at the
        // restore (NE555 point 13.3's pulse formula; the stepped run carries
        // integration error, so the sequence and rough timing are asserted).
        let vs = 5.0_f64;
        let expected = (ra + rb) * c * ((vs - v_held) / (vs - 2.0 * vs / 3.0)).ln();
        engine.advance(t_restore + 3.0 * expected).unwrap();
        let fall = slot0_trace(&engine).into_iter().rev().find(|&(_, v)| v == 0.0).map(|(t, _)| t).expect("the 555 must switch LOW after the restore");
        let measured = fall - t_restore;
        println!("power-loss restore: fall {measured:e} s after restore, expected {expected:e} s");
        assert!(((measured - expected) / expected).abs() < 0.02, "fall {measured} vs expected {expected}");
    }

    // ---- item 10: stepped reproduces the same digital sequence ------------

    #[test]
    fn stepped_partition_reproduces_the_same_digital_sequence_as_closed_form() {
        let edges = |force_stepped: bool| -> Vec<(f64, f64)> {
            let p = astable_program(1000.0, 1000.0, 1e-6, force_stepped);
            let mut e = Engine::load(&p).unwrap();
            e.trace_enable(true);
            e.advance(0.02).unwrap();
            slot0_trace(&e)
        };

        let cf = edges(false);
        let st = edges(true);
        assert!((cf.len() as i64 - st.len() as i64).abs() <= 1, "closed-form and stepped must take about the same number of transitions in this window: {} vs {}", cf.len(), st.len());
        for (i, (&(_, cf_v), &(_, st_v))) in cf.iter().zip(st.iter()).enumerate() {
            assert_eq!(cf_v, st_v, "transition {i} must alternate the same way (HIGH/LOW) in both");
        }

        // Period error: reported, not gated (the sequence match above is the
        // gate, per the brief).
        let cf_rises: Vec<f64> = cf.iter().filter(|&&(_, v)| v == 1.0).map(|&(t, _)| t).collect();
        let st_rises: Vec<f64> = st.iter().filter(|&&(_, v)| v == 1.0).map(|&(t, _)| t).collect();
        if cf_rises.len() >= 2 && st_rises.len() >= 2 {
            let cf_period = cf_rises[1] - cf_rises[0];
            let st_period = st_rises[1] - st_rises[0];
            let rel_err = ((st_period - cf_period) / cf_period).abs();
            println!("stepped vs closed-form period relative error: {rel_err:e} (closed-form {cf_period}, stepped {st_period})");
            assert!(rel_err < 0.05, "stepped period should be reasonably close to closed-form: {rel_err}");
        }
    }

    // ---- item 11: bucket exclusion -----------------------------------------

    #[test]
    fn bucket_exclusion_schedules_no_floating_bucket_for_a_555_read_pin_in_the_generic_dead_band() {
        // TRIG sits at a static 2.0V -- inside the GENERIC dead band
        // (max(0.8,0.3*5)=1.5, max(2.0,0.6*5)=3.0) that a W1c IC's own
        // FLOATING_INPUT/dead-band bucket watch would fire on every 10ms
        // (contract point 6/C8) -- a 555 must never register for it.
        let p = program(5, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 2, 3000.0),
            resistor(2, 2, 0, 2000.0),
            resistor(3, 3, 0, 1000.0),
            ne555_dev(4, 1, 0, [pin(2), pin(3), pin(1), pin(0), pin(0), pin(4)], 0),
        ]);
        assert!((2.0_f64 - 5.0 * 2000.0 / 5000.0).abs() < 1e-12, "sanity: the divider must land exactly on 2.0V");

        let mut engine = Engine::load(&p).unwrap();
        engine.advance(1.0).unwrap();
        assert_eq!(engine.stat(4), 0.0, "no event (a bucket or otherwise) should ever have been scheduled");
    }

    // ---- item 12: the datum ------------------------------------------------

    #[test]
    fn out_on_the_datum_node_stamps_nothing_but_still_publishes_the_formula() {
        // 0=gnd,1=vcc,2=dis(isolated). OUT is pin 3, tied straight to node 0.
        let p = program(3, 1, vec![
            source(0, 1, 0, 5.0),
            ne555_dev(1, 1, 0, [pin(0), pin(0), pin(1), pin(0), pin(0), pin(2)], 0),
        ]);
        let engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 1.0, "TRIG on GND sets OUT HIGH");

        let expected_i = element_current(true, 5.0, 0.0, 0.0, 5.0);
        assert!((expected_i - 0.245).abs() < 1e-12, "sanity: E question 4's own stated phantom value");
        assert!((engine.currents()[1] - expected_i).abs() < 1e-12, "elementI must still publish the formula even though nothing was stamped");
        assert!(engine.currents()[0].abs() < 1e-9, "the source must draw nothing: OUT-on-datum stamps into no real branch at all");
    }

    // ---- item 13: rollback through a zero-delay loop -----------------------

    #[test]
    fn zero_delay_loop_through_the_555_livelocks_and_rolls_back_cleanly() {
        // 0=gnd,1=vcc,2=x(TRIG=THR, 10k to VCC, no capacitor: OUT LOW,
        // steady),3=out,4=dis(isolated),5=the switch's far pin.
        let mut p = program(6, 1, vec![
            source(0, 1, 0, 5.0),
            resistor(1, 1, 2, 10_000.0),
            switch_dev(2, 3, 5, 0.0), // the loop-closing switch, initially OPEN.
            resistor(3, 5, 2, 1000.0),
            ne555_dev(4, 1, 0, [pin(2), pin(3), pin(1), pin(0), pin(2), pin(4)], 0),
        ]);

        let mut engine = Engine::load(&p).unwrap();
        assert_eq!(engine.digital_slots()[0], 0.0, "THR (=TRIG, floating up to VCC with no current path) resets OUT LOW at load, steady");
        assert!((engine.voltages()[2] - 5.0).abs() < 1e-6, "x floats to VCC (less the 1e-12 S shunt's drop across 10k) with no current path yet");

        let before_voltages = engine.voltages().to_vec();
        let before_currents = engine.currents().to_vec();
        let before_slot = engine.digital_slots()[0];
        let before_time = engine.time();

        let err = engine.set_params(&[(2, 0, 1.0)]).unwrap_err();
        assert_eq!(err.code, -7, "a zero-delay loop through the 555 must livelock (ERR_DELTA_LIVELOCK), not silently exit");

        assert_eq!(engine.time(), before_time, "a failed set_params must not move the clock");
        assert_eq!(engine.digital_slots()[0], before_slot, "the latch must be exactly as before the call");
        assert_eq!(engine.voltages(), before_voltages.as_slice(), "voltages must roll back exactly");
        assert_eq!(engine.currents(), before_currents.as_slice(), "currents must roll back exactly");

        // The same circuit LOADED with the switch already closed fails the
        // load itself with -7 (classic's own fallback rule, W1c).
        p.devices[2].params[0] = 1.0;
        let load_err = Engine::load(&p).unwrap_err();
        assert_eq!(load_err.code, -7, "loading with the loop already closed must also livelock");
    }
}

// ---- W2a: heap bytes (generated by the size walk generator; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Ne555State {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.out_high, w); }
        n
    }
}
