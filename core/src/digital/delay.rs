//! Maps one [`combinational::evaluate`] result onto the driver-level requests
//! the event kernel applies (core/README.md, "Kernel semantics: the digital
//! core (W1c)", point 7, and "...: datasheet delays and enables (W1d)",
//! points 1-6; `core/spec/w1-classic-semantics.md` §2.4, §3.7-3.8). Owned by
//! W1c worker C2; retrofitted for W1d datasheet delays by worker R8.
//!
//! `evaluate`'s `None`/`Some` result and `driver_requests`' own `reads`/
//! `last_reads` cause detection are the only combinational-family pieces
//! that changed for W1d: the FSM itself (`drivers::fsm`), `DelaySet` and the
//! `Level`/`Hiz` vocabulary were already in place (K1's kernel-infrastructure
//! wave).

use crate::drivers::DelaySet;
use crate::events::{DriverId, Mode};
use crate::kernel::DriverRequest;
use crate::kinds;

use super::IcRecord;

/// The [`DelaySet`] every output-capable pin of `ic` uses this evaluation,
/// from `ic`'s own delay tail (contract, "datasheet delays and enables
/// (W1d)", point 3's table) and, for the 157, which pin (`output_index`,
/// `output_capable(ic.kind)`'s index — 0..4, one per mux group) is being
/// requested.
///
/// - The nine kinds with no tail (`ic.delays` empty) get [`DelaySet::uniform`]
///   `(ic.delay)`: unchanged W1c timing, identical for every pin.
/// - **74ls245**: `lh`/`hl` are always param 12 (data); `zh`/`zl` are the
///   `oeEnable` pair and `hz`/`lz` the `oeDisable` pair, whichever cause (an
///   `/oe` edge or a `dir` flip) produced the HIZ transition — the kind has
///   only one enable pair and one disable pair, so there is nothing to pick
///   between. `zh`/`zl`/`hz`/`lz` never apply to `lh`/`hl` requests and vice
///   versa: `fsm::pick` only reads the pair for the transition actually
///   requested (contract point 2).
/// - **74ls157**: the strobe `/oe` is not an enable/disable pin (Q-O64: the
///   part is always driven, never HIZ), so its one tail pair (`strobe`)
///   times `lh`/`hl` instead. `lastReads` identifies the cause per the
///   contract's cause rule (point 3): `strobe` when this group's `/oe` read
///   differs from `last_reads`, else param 12; when a `sel` or this group's
///   own `a`/`b` read ALSO changed, the larger figure per direction wins
///   (point 3: "a datasheet's output-valid time is the latest of its
///   constraints"). No `last_reads` (the first powered evaluation) is no
///   identified cause, so param 12 — not `strobe`, `ic.delays[0]`'s literal
///   position notwithstanding: the contract's per-kind table lists param 12
///   as this kind's baseline cause and `strobe` as the exception.
fn delay_set(ic: &IcRecord, reads: &[bool], last_reads: Option<&[bool]>, output_index: usize) -> DelaySet {
    match ic.kind {
        kinds::IC_74LS245 => {
            let (zh, zl) = ic.delays[0]; // oeEnable
            let (hz, lz) = ic.delays[1]; // oeDisable
            DelaySet { lh: ic.delay, hl: ic.delay, zh, zl, hz, lz }
        }
        kinds::IC_74LS157 => {
            let (strobe_h, strobe_l) = ic.delays[0]; // strobe: dH release, dL force
            let changed = |i: usize| last_reads.is_some_and(|lr| lr[i] != reads[i]);
            let oe_changed = changed(1);
            let sel_changed = changed(0);
            let group = output_index; // 1y..4y -> groups 0..3, README's icPinMap order
            let data_changed = changed(2 + group * 3) || changed(3 + group * 3);
            let (lh, hl) = if oe_changed && (sel_changed || data_changed) {
                (strobe_h.max(ic.delay), strobe_l.max(ic.delay))
            } else if oe_changed {
                (strobe_h, strobe_l)
            } else {
                (ic.delay, ic.delay)
            };
            // The 157 never goes HIZ (Q-O64): these fields are unread by
            // `fsm::pick` for this kind, kept at the neutral param-12 figure
            // rather than left meaningless.
            DelaySet { lh, hl, zh: ic.delay, zl: ic.delay, hz: ic.delay, lz: ic.delay }
        }
        _ => DelaySet::uniform(ic.delay),
    }
}

/// Turns `result` (an [`combinational::evaluate`] return, one entry per
/// `output_capable(ic.kind)` pin, `drivers[i]` its driver) into the requests
/// `Kernel`'s delta cycle applies: every entry reaches its level — `Some`
/// HIGH/LOW, or `None` as [`Mode::Hiz`] (contract, "datasheet delays and
/// enables (W1d)", point 4: "tri-state is a HIZ request, never a reset" —
/// only the 74ls245 ever produces `None` now that the 157's strobe drives
/// every `Y`, Q-O64) — through the inertial FSM at the [`delay_set`] this
/// pin's cause picks out. `reads` is this evaluation's logic-pin reads (the
/// same slice `combinational::evaluate` was called with) and `last_reads`
/// this IC's reads at its previous powered evaluation (`None` before the
/// first), both needed only for the 157's cause rule.
///
/// Open-collector release is stamped by the SOLVER, not requested here: a
/// released 74LS47 segment is `Some(true)` in `result` (the evaluator already
/// inverted sinking to electrical HIGH — `combinational.rs`'s own doc), so it
/// takes the ordinary `Level { level: Mode::High, .. }` branch below exactly
/// like a push-pull HIGH. The solver's output-stage stamp is what turns a
/// committed HIGH on an `OPEN_COLLECTOR` pin into "stamp nothing" instead of
/// "stamp `1/R_out` to `vcc`" (contract point 8) — this function has no
/// `open_collector` branch on purpose.
pub fn driver_requests(
    ic: &IcRecord,
    drivers: &[DriverId],
    result: &[Option<bool>],
    reads: &[bool],
    last_reads: Option<&[bool]>,
) -> Vec<DriverRequest> {
    debug_assert_eq!(drivers.len(), result.len(), "one driver per output-capable pin, in the same order");
    drivers
        .iter()
        .zip(result.iter())
        .enumerate()
        .map(|(i, (&driver, level))| {
            let delays = delay_set(ic, reads, last_reads, i);
            let mode = match level {
                Some(true) => Mode::High,
                Some(false) => Mode::Low,
                None => Mode::Hiz,
            };
            DriverRequest::Level { driver, level: mode, delays }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::IcPin;

    /// A minimal `IcRecord` naming only what `driver_requests`/`delay_set`
    /// read (`kind`, `delay`, `delays`) — every other field belongs to
    /// `power.rs`/`levels.rs`'s read/power path, not this mapping, so it is
    /// zeroed/defaulted.
    fn ic_with(kind: u32, delay: f64, delays: Vec<(f64, f64)>) -> IcRecord {
        let no_pin = IcPin {
            node: 0,
            open: false,
            open_high: false,
            open_hash: 0,
            dead_hash: 0,
            floating_input: false,
            open_collector: false,
        };
        IcRecord {
            kind,
            vcc: no_pin,
            gnd: no_pin,
            logic: Vec::new(),
            vcc_min: 0.0,
            vcc_max: 0.0,
            ttl: false,
            cmos_ratio: false,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 0.0,
            io_max: None,
            delay,
            slot_base: 0,
            delays,
            extra: Vec::new(),
        }
    }

    /// A tailless kind (any of the nine W1c kinds without a delay tail):
    /// `driver_requests` never reads `reads`/`last_reads` for these, so the
    /// non-157/245 tests below pass empty/`None` for both.
    fn ic_with_delay(delay: f64) -> IcRecord {
        ic_with(crate::kinds::IC_74LS00, delay, Vec::new())
    }

    #[test]
    fn a_high_result_requests_level_high_at_the_ics_delay() {
        let ic = ic_with_delay(20e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(true)], &[], None);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::High, delays: DelaySet::uniform(20e-9) }]);
    }

    #[test]
    fn a_low_result_requests_level_low_at_the_ics_delay() {
        let ic = ic_with_delay(35e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(false)], &[], None);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::Low, delays: DelaySet::uniform(35e-9) }]);
    }

    #[test]
    fn an_absent_result_requests_hiz_instead_of_resetting_the_driver() {
        // W1d point 4: "tri-state is a HIZ request, never a reset" -- no IC
        // requests DriverRequest::Reset any more.
        let ic = ic_with_delay(20e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[None], &[], None);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::Hiz, delays: DelaySet::uniform(20e-9) }]);
    }

    #[test]
    fn an_open_collector_high_still_requests_level_high_release_is_the_solvers_job() {
        // The doc comment's claim, exercised directly: driver_requests has no
        // open_collector branch, so a released 74LS47 segment (Some(true))
        // takes the exact same path as an ordinary push-pull HIGH. Whether
        // that commit then stamps 1/R_out or nothing is the solver's read of
        // the pin's own `open_collector` flag at STAMP time, not something
        // this function ever inspects.
        let ic = ic_with_delay(35e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(true)], &[], None);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::High, delays: DelaySet::uniform(35e-9) }]);
    }

    #[test]
    fn maps_several_pins_in_order_matching_drivers_to_results_positionally() {
        let ic = ic_with_delay(0.0);
        let drivers = [DriverId(0), DriverId(1), DriverId(2), DriverId(3)];
        let result = [Some(true), None, Some(false), Some(true)];
        let reqs = driver_requests(&ic, &drivers, &result, &[], None);
        assert_eq!(
            reqs,
            vec![
                DriverRequest::Level { driver: DriverId(0), level: Mode::High, delays: DelaySet::uniform(0.0) },
                DriverRequest::Level { driver: DriverId(1), level: Mode::Hiz, delays: DelaySet::uniform(0.0) },
                DriverRequest::Level { driver: DriverId(2), level: Mode::Low, delays: DelaySet::uniform(0.0) },
                DriverRequest::Level { driver: DriverId(3), level: Mode::High, delays: DelaySet::uniform(0.0) },
            ]
        );
    }

    // ---- 74ls245: lh/hl always param 12, zh/zl/hz/lz always the oe pairs --

    fn ic_245() -> IcRecord {
        // param 12 = 20 ns; oeEnable (zh, zl) = 25/27 ns; oeDisable (hz, lz) = 15/15 ns
        // (core/README.md's "IC delay tail" typical figures).
        ic_with(crate::kinds::IC_74LS245, 20e-9, vec![(25e-9, 27e-9), (15e-9, 15e-9)])
    }

    #[test]
    fn a_245_level_request_carries_param_12_for_lh_hl_and_the_oe_pairs_for_enable_disable() {
        let ic = ic_245();
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(true)], &[], None);
        let expected = DelaySet { lh: 20e-9, hl: 20e-9, zh: 25e-9, zl: 27e-9, hz: 15e-9, lz: 15e-9 };
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::High, delays: expected }]);
    }

    #[test]
    fn a_245_hiz_request_carries_the_same_delay_set_as_a_level_request() {
        // driver_requests never picks which of zh/zl/hz/lz applies -- that is
        // fsm::pick's job, downstream, once it knows the committed level. It
        // only has to hand over the full set, unchanged by which pin/side is
        // undriven this evaluation.
        let ic = ic_245();
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[None], &[], None);
        let expected = DelaySet { lh: 20e-9, hl: 20e-9, zh: 25e-9, zl: 27e-9, hz: 15e-9, lz: 15e-9 };
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::Hiz, delays: expected }]);
    }

    // ---- 74ls157: strobe cause detection against lastReads ----------------

    // Pin order: sel(0) /oe(1) 1a(2) 1b(3) 1y(4) 2a(5) 2b(6) 2y(7) 3a(8) 3b(9)
    // 3y(10) 4a(11) 4b(12) 4y(13) -- README's icPinMap order, mirrored by
    // combinational.rs's mux157.
    fn ic_157() -> IcRecord {
        // param 12 = 20 ns; strobe (dH release, dL force) = 13/14 ns.
        ic_with(crate::kinds::IC_74LS157, 20e-9, vec![(13e-9, 14e-9)])
    }

    fn levels157(sel: bool, oe: bool, ab: [(bool, bool); 4]) -> [bool; 14] {
        let mut l = [false; 14];
        l[0] = sel;
        l[1] = oe;
        for (g, &(a, b)) in ab.iter().enumerate() {
            l[2 + g * 3] = a;
            l[3 + g * 3] = b;
        }
        l
    }

    #[test]
    fn mux157_first_powered_evaluation_has_no_cause_so_every_group_gets_param_12() {
        // No last_reads at all: contract point 3's "no last_reads (first
        // powered evaluation) means no identified cause, so param 12" --
        // not the strobe pair, even though it is this kind's only tail pair.
        let ic = ic_157();
        let reads = levels157(false, true, [(true, false); 4]); // /oe HIGH: strobed
        let drivers = [DriverId(0), DriverId(1), DriverId(2), DriverId(3)];
        let result = [Some(false), Some(false), Some(false), Some(false)];
        let reqs = driver_requests(&ic, &drivers, &result, &reads, None);
        for req in &reqs {
            match req {
                DriverRequest::Level { delays, .. } => assert_eq!(*delays, DelaySet::uniform(20e-9)),
                other => panic!("expected a Level request, got {other:?}"),
            }
        }
    }

    #[test]
    fn mux157_oe_changed_alone_uses_the_strobe_pair_for_every_group() {
        let ic = ic_157();
        let last = levels157(false, false, [(false, false); 4]); // /oe LOW
        let reads = levels157(false, true, [(false, false); 4]); // /oe rose, nothing else moved
        let drivers = [DriverId(0), DriverId(1), DriverId(2), DriverId(3)];
        let result = [Some(false), Some(false), Some(false), Some(false)];
        let reqs = driver_requests(&ic, &drivers, &result, &reads, Some(&last));
        for req in &reqs {
            match req {
                DriverRequest::Level { delays, .. } => {
                    assert_eq!(*delays, DelaySet { lh: 13e-9, hl: 14e-9, zh: 20e-9, zl: 20e-9, hz: 20e-9, lz: 20e-9 });
                }
                other => panic!("expected a Level request, got {other:?}"),
            }
        }
    }

    #[test]
    fn mux157_data_changed_without_oe_uses_param_12_not_strobe() {
        let ic = ic_157();
        let last = levels157(false, false, [(false, false); 4]);
        let reads = levels157(false, false, [(true, false), (false, false), (false, false), (false, false)]);
        let drivers = [DriverId(0)];
        let result = [Some(true)];
        let reqs = driver_requests(&ic, &drivers, &result, &reads, Some(&last));
        assert_eq!(reqs, vec![DriverRequest::Level { driver: DriverId(0), level: Mode::High, delays: DelaySet::uniform(20e-9) }]);
    }

    #[test]
    fn mux157_oe_and_this_groups_data_both_changed_uses_the_larger_figure_per_direction() {
        // Here the datasheet's own 20 ns package delay (param 12) is larger
        // than either strobe figure, so it wins both directions -- the point
        // is that the code picks the larger, not that 20 always beats 13/14.
        let ic = ic_157();
        let last = levels157(false, false, [(false, false); 4]);
        let reads = levels157(false, true, [(true, false), (false, false), (false, false), (false, false)]);
        let drivers = [DriverId(0), DriverId(1)];
        let result = [Some(false), Some(false)];
        let reqs = driver_requests(&ic, &drivers, &result, &reads, Some(&last));
        let expected_group0 = DelaySet { lh: 20e-9, hl: 20e-9, zh: 20e-9, zl: 20e-9, hz: 20e-9, lz: 20e-9 };
        assert_eq!(reqs[0], DriverRequest::Level { driver: DriverId(0), level: Mode::Low, delays: expected_group0 });
        // Group 1's own data did not change, only the shared /oe: strobe alone.
        let expected_group1 = DelaySet { lh: 13e-9, hl: 14e-9, zh: 20e-9, zl: 20e-9, hz: 20e-9, lz: 20e-9 };
        assert_eq!(reqs[1], DriverRequest::Level { driver: DriverId(1), level: Mode::Low, delays: expected_group1 });
    }

    #[test]
    fn mux157_sel_changed_counts_as_a_shared_cause_like_data() {
        // sel is read by every group, so an /oe + sel combination takes the
        // "both a level cause and the strobe fired" branch for every group,
        // even though no single group's own a/b pins moved.
        let ic = ic_157();
        let last = levels157(false, false, [(false, false); 4]);
        let reads = levels157(true, true, [(false, false); 4]);
        let drivers = [DriverId(0)];
        let result = [Some(false)];
        let reqs = driver_requests(&ic, &drivers, &result, &reads, Some(&last));
        assert_eq!(
            reqs,
            vec![DriverRequest::Level {
                driver: DriverId(0),
                level: Mode::Low,
                delays: DelaySet { lh: 20e-9, hl: 20e-9, zh: 20e-9, zl: 20e-9, hz: 20e-9, lz: 20e-9 },
            }]
        );
    }
}
