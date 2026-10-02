//! The `output_sag` target rule, predicate and window (G2a): core/README.md,
//! W1g kernel semantics, point 9 (classic SE:6274-6365, targets SE:6375-6555,
//! with the direction-aware debounce of `7b1056d`).
//!
//! Pure functions and plain `Clone` data, no solver types and no journal of
//! its own: G2b owns the scan (`settle`), the window event and the rollback,
//! and puts a [`SagTable`] under G1's instant journal. This module decides
//! three things from values the caller already holds:
//! - which direction a driver is commanded, if any ([`target`]);
//! - whether the pin sags in that direction at a given voltage ([`read`]);
//! - what state the window is in after an observation or the window's own
//!   event ([`SagState::observe`], [`SagState::on_window`]).

use crate::digital::power::IcPower;
use crate::drivers::{Driver, DriverStage};
use crate::events::Mode;
use crate::slow::failures::{FailureKind, FailureRecord};

/// How long an `output_sag` must persist in one direction before it is
/// published, seconds (W1g Q2, Q-O43). Classic latches at two consecutive
/// accepted steps in one direction (`OUTPUT_SAG_DEBOUNCE_STEPS = 2`, SE:131),
/// and two accepted steps are at least `H_MIN = 1e-8` apart.
pub const OUTPUT_SAG_WINDOW: f64 = 1e-8;

/// A window event within this of its due time counts as due, the contract's
/// convention for every slow event (point 5: "`t_x <= now + 1e-15`").
const DUE_EPS: f64 = 1e-15;

/// The commanded direction of a driven pin, and the `aux` of its record
/// (`1` HIGH, `0` LOW).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Low,
    High,
}

impl Dir {
    pub fn aux(self) -> f64 {
        match self {
            Dir::High => 1.0,
            Dir::Low => 0.0,
        }
    }
}

/// The target rule (point 9): for a powered IC, a driver whose stage is
/// `Logic` or `Ne555Out` and whose committed mode is `HIGH` or `LOW` is a
/// target, except an `OPEN_COLLECTOR` pin committed `HIGH`, which is released.
/// A pin committed `HIZ`, never committed, or on an unpowered IC is not.
///
/// One rule covers every W1 and W1h kind's per-kind table in classic: the
/// 555's pin 3 and never its DIS (`Ne555Dis`), a 74LS47 segment only while it
/// sinks (Q-O49, `d1dee17`), the disabled 173, 189, EEPROM and 595 outputs
/// (committed `HIZ`) and the 74LS245's driven side.
pub fn target(stage: DriverStage, committed: Option<Mode>, open_collector: bool, powered: bool) -> Option<Dir> {
    if !powered {
        return None;
    }
    match stage {
        DriverStage::Logic | DriverStage::Ne555Out => {}
        DriverStage::Ne555Dis => return None,
    }
    match committed {
        Some(Mode::High) if open_collector => None,
        Some(Mode::High) => Some(Dir::High),
        Some(Mode::Low) => Some(Dir::Low),
        Some(Mode::Hiz) | None => None,
    }
}

/// [`target`] for a registered driver.
pub fn driver_target(driver: &Driver, powered: bool) -> Option<Dir> {
    target(driver.stage, driver.committed(), driver.open_collector, powered)
}

/// A pin that sags in the commanded direction, with what its record carries:
/// `actual = V(pin) - V(gnd)` and the threshold it missed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub dir: Dir,
    pub actual: f64,
    /// `vih` for a HIGH sag, `vil` for a LOW one, at the live supply.
    pub limit: f64,
}

impl Reading {
    /// The `output_sag` record at `since`. `pin` is the pin's index in the
    /// device's pin records (an IC's logic pin `i` is `i + 2`, after `vcc` and
    /// `gnd`).
    pub fn record(&self, device: usize, pin: usize, since: f64) -> FailureRecord {
        FailureRecord { device, kind: FailureKind::OutputSag, pin: Some(pin), since, value: self.actual, limit: self.limit, aux: self.dir.aux() }
    }
}

/// The predicate (point 9), strict as classic's (SE:6304-6306): a commanded
/// HIGH sags when `actual < vih`, a commanded LOW when `actual > vil`, with
/// `actual = V(pin) - V(gnd)` and the thresholds at the live `vS`
/// (`digital::power::ic_power`). `None` while the IC is unpowered, so a window
/// event never latches a sag whose supply has since gone.
pub fn read(dir: Dir, v_pin: f64, power: &IcPower) -> Option<Reading> {
    if !power.powered {
        return None;
    }
    let actual = v_pin - power.v_gnd;
    let (sags, limit) = match dir {
        Dir::High => (actual < power.vih, power.vih),
        Dir::Low => (actual > power.vil, power.vil),
    };
    sags.then_some(Reading { dir, actual, limit })
}

/// The direction a scan observes at one pin: the commanded direction when it
/// sags, otherwise none.
pub fn observed(target: Option<Dir>, v_pin: f64, power: &IcPower) -> Option<Dir> {
    target.and_then(|dir| read(dir, v_pin, power)).map(|r| r.dir)
}

/// The window state of one `(IC, pin)` (W1g Q2). Only `Latched` is published.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum SagState {
    #[default]
    Ok,
    /// A sag in `dir` first seen at `t_s`, not yet persistent.
    Pending { dir: Dir, t_s: f64 },
    Latched { dir: Dir },
}

/// Whether a transition changed what is published, which is what moves a
/// record in or out of the failure map (and so its place in latch order,
/// `bo_engine_records` "Order").
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Publish {
    Unchanged,
    /// `Latched` was entered: a new latch, with the event's time as `since`
    /// for the latch log.
    Latched,
    /// A published latch ended: a clear, or an unpublished direction flip.
    Unlatched,
}

/// What one transition asks of the caller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transition {
    /// The time of the window event to schedule, whenever the state after the
    /// transition is `Pending`. It is returned again for an unchanged
    /// `Pending`, so a caller whose device token moved (a publish bumps it,
    /// point 5) can reschedule; a value already due is latched at once by the
    /// caller's `now + 1e-15` rule.
    pub schedule: Option<f64>,
    pub publish: Publish,
}

/// The end of the window that opened at `t_s`.
pub fn window_end(t_s: f64) -> f64 {
    t_s + OUTPUT_SAG_WINDOW
}

impl SagState {
    /// The direction of the published latch, if any.
    pub fn published(&self) -> Option<Dir> {
        match *self {
            SagState::Latched { dir } => Some(dir),
            _ => None,
        }
    }

    /// One scan observation at `now`: `sag` is the direction the pin sags in
    /// ([`observed`]), or none (point 9, "At each scan").
    /// - no sag: `Ok`, which clears a latch;
    /// - a sag in `dir` while `Ok`, or while `Pending` or `Latched` in the
    ///   other direction: `Pending { dir, now }`. Restarting on a flip is
    ///   classic's direction-aware debounce (`7b1056d`: "a flip restarts it at
    ///   1, and below 2 the failure is not published"), so a latched sag whose
    ///   direction flips while the pin still sags is unpublished at once;
    /// - a sag in `dir` while `Pending { dir }` or `Latched { dir }`: no
    ///   change.
    pub fn observe(&mut self, now: f64, sag: Option<Dir>) -> Transition {
        let was = *self;
        let unlatched = if was.published().is_some() { Publish::Unlatched } else { Publish::Unchanged };
        match (was, sag) {
            (_, None) => {
                *self = SagState::Ok;
                Transition { schedule: None, publish: unlatched }
            }
            (SagState::Pending { dir: d, t_s }, Some(dir)) if d == dir => Transition { schedule: Some(window_end(t_s)), publish: Publish::Unchanged },
            (SagState::Latched { dir: d }, Some(dir)) if d == dir => Transition { schedule: None, publish: Publish::Unchanged },
            (_, Some(dir)) => {
                *self = SagState::Pending { dir, t_s: now };
                Transition { schedule: Some(window_end(now)), publish: unlatched }
            }
        }
    }

    /// The window's own event at `now`: a current `Pending { dir, t_s }` whose
    /// window has run out and whose predicate still holds in `dir` (`holds`,
    /// read at `now`: [`read`] is `Some`) latches. If it no longer holds the
    /// sag did not last the window, so the state clears. An event that is not
    /// yet due (a stale one for an earlier `Pending`) and any other state are
    /// left alone.
    pub fn on_window(&mut self, now: f64, holds: bool) -> Transition {
        match *self {
            SagState::Pending { dir, t_s } => {
                let end = window_end(t_s);
                if now + DUE_EPS < end {
                    Transition { schedule: Some(end), publish: Publish::Unchanged }
                } else if holds {
                    *self = SagState::Latched { dir };
                    Transition { schedule: None, publish: Publish::Latched }
                } else {
                    *self = SagState::Ok;
                    Transition { schedule: None, publish: Publish::Unchanged }
                }
            }
            _ => Transition { schedule: None, publish: Publish::Unchanged },
        }
    }
}

/// One `(IC, pin)` entry of a [`SagTable`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SagCell {
    /// The IC's device index.
    pub device: usize,
    /// The pin's index in the device's pin records, as the record writes it.
    pub pin: usize,
    pub state: SagState,
}

/// Every driven pin's window state, one entry per `(IC, pin)` in the order
/// the caller registered them. Plain `Clone` data: G2b puts it under G1's
/// journal, which copies an entry or the whole table on first touch.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SagTable {
    cells: Vec<SagCell>,
}

impl SagTable {
    /// A table of `(device, pin)` keys, all `Ok`.
    pub fn new(keys: impl IntoIterator<Item = (usize, usize)>) -> SagTable {
        SagTable { cells: keys.into_iter().map(|(device, pin)| SagCell { device, pin, state: SagState::Ok }).collect() }
    }

    pub fn len(&self) -> usize {
        self.cells.len()
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    pub fn cell(&self, i: usize) -> SagCell {
        self.cells[i]
    }

    pub fn iter(&self) -> impl Iterator<Item = &SagCell> {
        self.cells.iter()
    }

    pub fn index_of(&self, device: usize, pin: usize) -> Option<usize> {
        self.cells.iter().position(|c| c.device == device && c.pin == pin)
    }

    /// Replaces entry `i`'s state, for restoring a journal copy.
    pub fn set_state(&mut self, i: usize, state: SagState) {
        self.cells[i].state = state;
    }

    pub fn observe(&mut self, i: usize, now: f64, sag: Option<Dir>) -> Transition {
        self.cells[i].state.observe(now, sag)
    }

    pub fn on_window(&mut self, i: usize, now: f64, holds: bool) -> Transition {
        self.cells[i].state.on_window(now, holds)
    }

    /// Every entry back to `Ok`, as a load does (`resetFailures` re-observes
    /// from the reset instant, point 9 "Load and reset").
    pub fn clear(&mut self) {
        self.cells.iter_mut().for_each(|c| c.state = SagState::Ok);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digital::power::ic_power;
    use crate::digital::{IcPin, IcRecord};
    use crate::drivers::DelaySet;
    use crate::kinds;

    fn pin(node: usize) -> IcPin {
        IcPin { node, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    /// The 74LS00 of probe F `sag`: TTL with the catalog's fixed `v_il_max`
    /// 0.8 and `v_ih_min` 2.0, `io_max` 8 mA.
    fn ls00() -> IcRecord {
        IcRecord {
            kind: kinds::IC_74LS00,
            vcc: pin(1),
            gnd: pin(0),
            logic: Vec::new(),
            vcc_min: 4.75,
            vcc_max: 5.25,
            ttl: true,
            cmos_ratio: false,
            vil: Some(0.8),
            vih: Some(2.0),
            schmitt: None,
            nominal: 5.0,
            io_max: Some(0.008),
            delay: 20e-9,
            slot_base: 0,
            delays: Vec::new(),
            extra: Vec::new(),
        }
    }

    /// The 74HC138 of probe S: HC-CMOS ratio thresholds (`vil = max(0.2, 0.3 vS)`
    /// = 1.5 V, `vih = max(0.4, 0.7 vS)` = 3.5 V at 5 V), `io_max` 25 mA.
    fn hc138() -> IcRecord {
        IcRecord { kind: kinds::IC_74HC138, ttl: false, cmos_ratio: true, vil: None, vih: None, io_max: Some(0.025), ..ls00() }
    }

    fn driver(stage: DriverStage, open_collector: bool, mode: Option<Mode>) -> Driver {
        let mut d = Driver::new_ic_output(5, 1, 0, 0, open_collector);
        d.stage = stage;
        if let Some(mode) = mode {
            // Through the real FSM: the first request commits at once.
            let (_, m, token) = d.request(mode, 0.0, &DelaySet::uniform(0.0)).expect("a first request schedules");
            assert!(d.apply_commit(m, token));
            assert_eq!(d.committed(), Some(mode));
        }
        d
    }

    // ---- the target rule -----------------------------------------------------

    #[test]
    fn a_74ls00_output_is_a_target_when_committed_high_or_low_and_not_when_hiz() {
        let high = driver(DriverStage::Logic, false, Some(Mode::High));
        let low = driver(DriverStage::Logic, false, Some(Mode::Low));
        let hiz = driver(DriverStage::Logic, false, Some(Mode::Hiz));
        let never = driver(DriverStage::Logic, false, None);
        assert_eq!(driver_target(&high, true), Some(Dir::High));
        assert_eq!(driver_target(&low, true), Some(Dir::Low));
        assert_eq!(driver_target(&hiz, true), None);
        assert_eq!(driver_target(&never, true), None);
    }

    #[test]
    fn a_74ls47_segment_is_a_target_only_while_it_sinks() {
        // Q-O49 (`d1dee17`): an `OPEN_COLLECTOR` pin committed HIGH is released.
        let mut ls47 = ls00();
        ls47.kind = kinds::IC_74LS47;
        ls47.logic = (0..14).map(pin).collect();
        for &i in crate::digital::output_capable(kinds::IC_74LS47) {
            ls47.logic[i].open_collector = true;
        }
        let seg = &ls47.logic[crate::digital::output_capable(kinds::IC_74LS47)[0]];
        assert!(seg.open_collector);
        let released = driver(DriverStage::Logic, seg.open_collector, Some(Mode::High));
        let sinking = driver(DriverStage::Logic, seg.open_collector, Some(Mode::Low));
        assert_eq!(driver_target(&released, true), None, "a released segment is not a target");
        assert_eq!(driver_target(&sinking, true), Some(Dir::Low));
        // A push-pull pin committed HIGH, on the same IC, still is.
        assert_eq!(driver_target(&driver(DriverStage::Logic, false, Some(Mode::High)), true), Some(Dir::High));
    }

    #[test]
    fn the_555_gives_pin_3_and_never_pin_7() {
        for mode in [Mode::High, Mode::Low, Mode::Hiz] {
            assert_eq!(driver_target(&driver(DriverStage::Ne555Dis, false, Some(mode)), true), None, "DIS, {mode:?}");
        }
        assert_eq!(driver_target(&driver(DriverStage::Ne555Out, false, Some(Mode::High)), true), Some(Dir::High));
        assert_eq!(driver_target(&driver(DriverStage::Ne555Out, false, Some(Mode::Low)), true), Some(Dir::Low));
        assert_eq!(driver_target(&driver(DriverStage::Ne555Out, false, Some(Mode::Hiz)), true), None);
    }

    #[test]
    fn an_unpowered_ic_gives_no_target() {
        for mode in [Mode::High, Mode::Low] {
            assert_eq!(driver_target(&driver(DriverStage::Logic, false, Some(mode)), false), None);
            assert_eq!(driver_target(&driver(DriverStage::Ne555Out, false, Some(mode)), false), None);
        }
        let power = ic_power(&ls00(), 0.0, 0.0);
        assert!(!power.powered);
        assert_eq!(read(Dir::Low, 3.0, &power), None, "an unpowered IC has nothing to sag");
    }

    // ---- the predicate -------------------------------------------------------

    #[test]
    fn the_predicate_is_strict_at_the_live_thresholds() {
        let power = ic_power(&ls00(), 5.0, 0.0);
        assert_eq!((power.vil, power.vih), (0.8, 2.0));
        assert!(read(Dir::High, 2.0 - 1e-9, &power).is_some());
        assert_eq!(read(Dir::High, 2.0, &power), None, "HIGH at exactly vih does not sag");
        assert!(read(Dir::Low, 0.8 + 1e-9, &power).is_some());
        assert_eq!(read(Dir::Low, 0.8, &power), None, "LOW at exactly vil does not sag");
        assert_eq!(read(Dir::High, f64::NAN, &power), None);
    }

    #[test]
    fn the_pin_is_read_against_the_ic_ground_not_node_zero() {
        // gnd at 2 V, vcc at 7 V: vS = 5, a pin at 3.9 V is 1.9 V above gnd.
        let mut ic = ls00();
        ic.vcc.node = 2;
        ic.gnd.node = 1;
        let power = ic_power(&ic, 7.0, 2.0);
        let r = read(Dir::High, 3.9, &power).expect("1.9 V against a 2.0 V vih sags");
        assert!((r.actual - 1.9).abs() < 1e-12, "actual {}", r.actual);
        assert_eq!(r.limit, 2.0);
    }

    #[test]
    fn probe_f_sag_reads_1_9512195_volts_against_2_0() {
        // 74LS00 1Y commanded HIGH into 100 ohm to ground. R_out comes from
        // io_max (5 / (4 * 0.008) = 156.25) and the divider is the model's
        // closed form; probe F `sag` recorded 1.9512195 V and limit 2.0.
        let power = ic_power(&ls00(), 5.0, 0.0);
        assert!((power.r_out - 156.25).abs() < 1e-9, "r_out {}", power.r_out);
        let v = power.vs * 100.0 / (100.0 + power.r_out);
        let r = read(Dir::High, v, &power).expect("it sags");
        assert!((r.actual - 1.9512195).abs() < 1e-7, "actual {}", r.actual);
        assert_eq!(r.limit, 2.0);
        assert_eq!(r.dir.aux(), 1.0);
        // The record: [device, 4, pin(1y), since, value, limit, aux = 1], with the
        // pin's index in the device's pin records: `vcc` and `gnd` are 0 and 1, so
        // `1y` is its position in README's 74ls00 logic-pin list plus 2, which is
        // also the first of the kind's output-capable pins.
        let logic = ["1a", "1b", "1y", "2a", "2b", "2y", "3a", "3b", "3y", "4a", "4b", "4y"];
        let pin = logic.iter().position(|&p| p == "1y").unwrap() + 2;
        assert_eq!(pin, crate::digital::output_capable(kinds::IC_74LS00)[0] + 2);
        let rec = r.record(5, pin, 1e-8).to_f64s();
        assert_eq!(rec[..4], [5.0, 4.0, 4.0, 1e-8]);
        assert_eq!(rec[4..], [r.actual, 2.0, 1.0]);
    }

    #[test]
    fn a_low_sag_record_has_aux_zero_and_the_vil_limit() {
        let power = ic_power(&ls00(), 5.0, 0.0);
        let r = read(Dir::Low, 1.0, &power).expect("1.0 V against a 0.8 V vil sags");
        assert_eq!(r.limit, 0.8);
        assert_eq!(r.record(0, 3, 0.0).aux, 0.0);
    }

    // ---- the window ----------------------------------------------------------

    const SAG: Option<Dir> = Some(Dir::High);

    #[test]
    fn a_sag_present_at_load_is_pending_at_start_time_and_latches_one_window_later() {
        // Probe F `sag`: classic latches at 1e-8, the load seed counting as the
        // first observation and the first step being H_INITIAL = 1e-8.
        for start in [0.0, 3.0] {
            let mut s = SagState::Ok;
            let t = s.observe(start, SAG);
            assert_eq!(s, SagState::Pending { dir: Dir::High, t_s: start });
            assert_eq!(t, Transition { schedule: Some(start + 1e-8), publish: Publish::Unchanged });
            assert_eq!(s.published(), None, "pending is not published");

            let t = s.on_window(start + 1e-8, true);
            assert_eq!(s, SagState::Latched { dir: Dir::High });
            assert_eq!(t, Transition { schedule: None, publish: Publish::Latched });
            assert_eq!(s.published(), Some(Dir::High));
        }
    }

    #[test]
    fn a_window_event_before_it_is_due_does_nothing() {
        let mut s = SagState::Ok;
        s.observe(0.0, SAG);
        let t = s.on_window(0.5e-8, true);
        assert_eq!(s, SagState::Pending { dir: Dir::High, t_s: 0.0 });
        assert_eq!(t, Transition { schedule: Some(1e-8), publish: Publish::Unchanged });
    }

    #[test]
    fn a_repeat_observation_in_the_same_direction_keeps_the_window_open_time() {
        let mut s = SagState::Ok;
        s.observe(0.0, SAG);
        let t = s.observe(4e-9, SAG);
        assert_eq!(s, SagState::Pending { dir: Dir::High, t_s: 0.0 });
        assert_eq!(t.schedule, Some(1e-8), "re-announced so a bumped token can reschedule");
        s.on_window(1e-8, true);
        assert_eq!(s, SagState::Latched { dir: Dir::High });
        assert_eq!(s.observe(2e-8, SAG), Transition { schedule: None, publish: Publish::Unchanged });
        assert_eq!(s, SagState::Latched { dir: Dir::High });
    }

    #[test]
    fn no_sag_resets_a_pending_window() {
        let mut s = SagState::Ok;
        s.observe(0.0, SAG);
        let t = s.observe(5e-9, None);
        assert_eq!(s, SagState::Ok);
        assert_eq!(t, Transition { schedule: None, publish: Publish::Unchanged });
        // The first window's event arrives anyway (the heap keeps it): it finds
        // nothing to latch.
        s.on_window(1e-8, true);
        assert_eq!(s, SagState::Ok);
        // A sag that comes back opens a fresh window; the old event does not
        // shorten it.
        s.observe(1e-8, SAG);
        s.on_window(1.5e-8, true);
        assert_eq!(s, SagState::Pending { dir: Dir::High, t_s: 1e-8 });
        s.on_window(2e-8, true);
        assert_eq!(s, SagState::Latched { dir: Dir::High });
    }

    #[test]
    fn no_sag_clears_a_latch() {
        let mut s = SagState::Latched { dir: Dir::Low };
        let t = s.observe(1.0, None);
        assert_eq!(s, SagState::Ok);
        assert_eq!(t, Transition { schedule: None, publish: Publish::Unlatched });
    }

    #[test]
    fn a_pending_sag_that_no_longer_holds_at_its_event_clears() {
        // A closed-form segment can cross back inside the window without a
        // publish; the single reading at the event decides.
        let mut s = SagState::Ok;
        s.observe(0.0, SAG);
        let t = s.on_window(1e-8, false);
        assert_eq!(s, SagState::Ok);
        assert_eq!(t, Transition { schedule: None, publish: Publish::Unchanged });
    }

    #[test]
    fn a_direction_flip_before_the_window_ends_restarts_it() {
        let mut s = SagState::Ok;
        s.observe(0.0, Some(Dir::High));
        let t = s.observe(6e-9, Some(Dir::Low));
        assert_eq!(s, SagState::Pending { dir: Dir::Low, t_s: 6e-9 });
        assert_eq!(t, Transition { schedule: Some(window_end(6e-9)), publish: Publish::Unchanged });
        // The first window's event, at 1e-8, is early for the new window.
        s.on_window(1e-8, true);
        assert_eq!(s, SagState::Pending { dir: Dir::Low, t_s: 6e-9 });
        s.on_window(window_end(6e-9), true);
        assert_eq!(s, SagState::Latched { dir: Dir::Low });
    }

    /// A pin that sags in both directions (between `vil` and `vih`): the
    /// commanded direction alone decides which sag it is.
    fn between_the_thresholds() -> (IcPower, f64) {
        let power = ic_power(&ls00(), 5.0, 0.0);
        let v = 1.9512195;
        assert!(read(Dir::High, v, &power).is_some() && read(Dir::Low, v, &power).is_some());
        (power, v)
    }

    #[test]
    fn a_latched_high_sag_whose_direction_flips_while_it_still_sags_is_unpublished_and_relatches_low() {
        // `test/sim/engine/output-sag-direction.test.ts` at `7b1056d`, in the
        // window's terms: the count carries the direction, a flip restarts it
        // at 1, and below 2 the failure is not published.
        let (power, v) = between_the_thresholds();
        let mut s = SagState::Ok;
        s.observe(0.0, observed(Some(Dir::High), v, &power));
        s.on_window(1e-8, read(Dir::High, v, &power).is_some());
        assert_eq!(s, SagState::Latched { dir: Dir::High });

        // The commanded direction flips to LOW and the pin still sags.
        let t = s.observe(5e-8, observed(Some(Dir::Low), v, &power));
        assert_eq!(t.publish, Publish::Unlatched, "unpublished at once");
        assert_eq!(s.published(), None);
        assert_eq!(s, SagState::Pending { dir: Dir::Low, t_s: 5e-8 });

        // A scan in the middle of the new window leaves it pending.
        s.observe(5.5e-8, observed(Some(Dir::Low), v, &power));
        assert_eq!(s.published(), None);

        let t = s.on_window(window_end(5e-8), read(Dir::Low, v, &power).is_some());
        assert_eq!(t.publish, Publish::Latched);
        assert_eq!(s, SagState::Latched { dir: Dir::Low });
    }

    #[test]
    fn an_output_that_sags_in_both_directions_but_flips_every_5_ns_never_latches() {
        let (power, v) = between_the_thresholds();
        let mut s = SagState::Ok;
        let mut stale: Vec<f64> = Vec::new();
        for k in 0..400 {
            let now = k as f64 * 5e-9;
            // Every window event ever scheduled arrives when due, current or
            // not, as a kernel heap without tokens would deliver it.
            let (due, rest): (Vec<f64>, Vec<f64>) = stale.drain(..).partition(|&e| e <= now + 1e-15);
            stale = rest;
            for e in due {
                let holds = read(Dir::High, v, &power).is_some();
                assert_ne!(s.on_window(e, holds).publish, Publish::Latched, "latched at k = {k}");
            }
            let dir = if k % 2 == 0 { Dir::High } else { Dir::Low };
            let t = s.observe(now, observed(Some(dir), v, &power));
            if let Some(e) = t.schedule {
                stale.push(e);
            }
            assert_eq!(s.published(), None, "published at k = {k}");
        }
    }

    #[test]
    fn a_sustained_sag_latches_in_both_directions() {
        let (power, v) = between_the_thresholds();
        for dir in [Dir::High, Dir::Low] {
            let mut s = SagState::Ok;
            let t = s.observe(0.0, observed(Some(dir), v, &power));
            let t = s.on_window(t.schedule.expect("pending"), read(dir, v, &power).is_some());
            assert_eq!(t.publish, Publish::Latched);
            assert_eq!(s, SagState::Latched { dir });
        }
    }

    // ---- probe S -------------------------------------------------------------

    /// Draft section 8's table (probe S, OUT = 88, the display section of the
    /// SAP-1 on classic's HeadlessRunner), the U41 (74HC138) outputs `y0n` and
    /// `y2n` at each accepted step: the time, the committed direction and the
    /// pin voltage as classic solved it (under the level stamped at the START
    /// of the step, the one-solve lag of Q-O19), and the voltage the same
    /// level would give if it had been the one solved (what the core
    /// stamps).
    struct Row {
        t: f64,
        y0n: (Dir, f64, f64),
        y2n: (Dir, f64, f64),
    }

    fn probe_s_rows() -> [Row; 5] {
        // A lit digit sinks 1.1123 V (7 segments, "8"); an idle line is 5 V.
        // Rows 1-2: y0n HIGH (5 V), y2n HIGH. Row 3: the glitch commits y0n LOW
        // (classic still reads 5 V). Row 4: y0n back HIGH (classic still reads
        // 1.1123 V), y2n LOW committed (classic still reads 5 V). Row 5: y2n
        // reads its own 1.1123 V.
        let (lo, hi) = (1.1123, 5.0);
        [
            Row { t: 0.113054849997, y0n: (Dir::High, hi, hi), y2n: (Dir::High, hi, hi) },
            Row { t: 0.113054868841, y0n: (Dir::High, hi, hi), y2n: (Dir::High, hi, hi) },
            Row { t: 0.113054888841, y0n: (Dir::Low, hi, lo), y2n: (Dir::High, hi, hi) },
            Row { t: 0.113054908841, y0n: (Dir::High, lo, hi), y2n: (Dir::Low, hi, lo) },
            Row { t: 0.113054937108, y0n: (Dir::High, hi, hi), y2n: (Dir::Low, lo, lo) },
        ]
    }

    #[test]
    fn probe_s_lagged_flip_between_rows_3_and_4_opens_a_fresh_window() {
        // Before `7b1056d` the pin-keyed count read (commanded LOW, 5 V) then
        // (commanded HIGH, 1.1123 V) as two sags and latched y0n at row 4. This
        // claims only the scan transition: the second observation is in the
        // opposite direction, so it opens a fresh window instead of counting as
        // a second observation. It says nothing about whether either window
        // would latch if its event were delivered, which the contract's window rule
        // allows and this test does not claim.
        let power = ic_power(&hc138(), 5.0, 0.0);
        assert_eq!((power.vil, power.vih), (1.5, 3.5));
        let mut y0n = SagState::Ok;
        let rows = probe_s_rows();
        y0n.observe(rows[2].t, observed(Some(rows[2].y0n.0), rows[2].y0n.1, &power));
        assert_eq!(y0n, SagState::Pending { dir: Dir::Low, t_s: rows[2].t });
        y0n.observe(rows[3].t, observed(Some(rows[3].y0n.0), rows[3].y0n.1, &power));
        assert_eq!(y0n, SagState::Pending { dir: Dir::High, t_s: rows[3].t });
    }

    #[test]
    fn probe_s_at_the_stamped_level_has_no_sag_at_all() {
        // The core stamps what it commits (decision 7), so each row's voltage
        // is that of the level committed in it. Every lit digit sinks inside
        // the LOW band (vil 1.5 V) and every idle line sits above the HIGH
        // band's 3.5 V: no row is a sag, so no window ever opens and nothing
        // latches (draft section 8: "There is no real sag at any digit").
        let power = ic_power(&hc138(), 5.0, 0.0);
        let mut y0n = SagState::Ok;
        let mut y2n = SagState::Ok;
        for row in probe_s_rows() {
            for (state, (dir, _, v_stamped)) in [(&mut y0n, row.y0n), (&mut y2n, row.y2n)] {
                let t = state.observe(row.t, observed(Some(dir), v_stamped, &power));
                assert_eq!(*state, SagState::Ok, "t = {}", row.t);
                assert_eq!(t, Transition { schedule: None, publish: Publish::Unchanged });
            }
        }
    }

    // ---- the table -----------------------------------------------------------

    #[test]
    fn a_table_is_plain_clone_data_keyed_by_device_and_pin() {
        let mut table = SagTable::new([(3, 4), (3, 5), (7, 4)]);
        assert_eq!(table.len(), 3);
        assert_eq!(table.index_of(3, 5), Some(1));
        assert_eq!(table.index_of(7, 5), None);

        let before = table.clone();
        let t = table.observe(1, 0.0, Some(Dir::Low));
        assert_eq!(t.schedule, Some(1e-8));
        assert_eq!(table.cell(1).state, SagState::Pending { dir: Dir::Low, t_s: 0.0 });
        assert_eq!(table.cell(0).state, SagState::Ok);
        assert_ne!(table, before);

        // Restoring an entry from a copy taken before the change.
        table.set_state(1, before.cell(1).state);
        assert_eq!(table, before);

        table.observe(1, 0.0, Some(Dir::Low));
        table.clear();
        assert!(table.iter().all(|c| c.state == SagState::Ok));
        assert_eq!(table.cell(2), SagCell { device: 7, pin: 4, state: SagState::Ok });
    }
}
