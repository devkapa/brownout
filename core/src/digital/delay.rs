//! Maps one [`combinational::evaluate`] result onto the driver-level requests
//! the event kernel applies (core/README.md, "Kernel semantics: the digital
//! core (W1c)", point 7; `core/spec/w1-classic-semantics.md` §2.4, §3.7-3.8).
//! Owned by W1c worker C2.

use crate::events::{DriverId, Mode};
use crate::kernel::DriverRequest;

use super::IcRecord;

/// Turns `result` (an [`combinational::evaluate`] return, one entry per
/// `output_capable(ic.kind)` pin, `drivers[i]` its driver) into the requests
/// `Kernel`'s delta cycle applies: `Some(level)` reaches `level` through the
/// inertial FSM at the IC's own propagation delay (contract point 7 — "each
/// output-capable pin in the result requests its level through its driver");
/// `None` resets the driver outright (a 157/245 tri-stated by `/oe`, or the
/// undriven side of a 245), matching contract point 7's "a pin absent from
/// the result is reset: its driver goes HIZ and drops any in-flight
/// transition, so re-enabling re-seeds with zero delay".
///
/// Open-collector release is stamped by the SOLVER, not requested here: a
/// released 74LS47 segment is `Some(true)` in `result` (the evaluator already
/// inverted sinking to electrical HIGH — `combinational.rs`'s own doc), so it
/// takes the ordinary `Level { level: Mode::High, .. }` branch below exactly
/// like a push-pull HIGH. The solver's output-stage stamp is what turns a
/// committed HIGH on an `OPEN_COLLECTOR` pin into "stamp nothing" instead of
/// "stamp `1/R_out` to `vcc`" (contract point 8) — this function has no
/// `open_collector` branch on purpose.
pub fn driver_requests(ic: &IcRecord, drivers: &[DriverId], result: &[Option<bool>]) -> Vec<DriverRequest> {
    debug_assert_eq!(drivers.len(), result.len(), "one driver per output-capable pin, in the same order");
    drivers
        .iter()
        .zip(result.iter())
        .map(|(&driver, level)| match level {
            Some(true) => DriverRequest::Level { driver, level: Mode::High, delay: ic.delay },
            Some(false) => DriverRequest::Level { driver, level: Mode::Low, delay: ic.delay },
            None => DriverRequest::Reset { driver },
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::IcPin;

    /// A minimal `IcRecord` naming only what `driver_requests` reads
    /// (`delay`) — every other field belongs to `power.rs`/`levels.rs`'s
    /// read/power path, not this mapping, so it is zeroed/defaulted.
    fn ic_with_delay(delay: f64) -> IcRecord {
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
            kind: crate::kinds::IC_74LS00,
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
        }
    }

    #[test]
    fn a_high_result_requests_level_high_at_the_ics_delay() {
        let ic = ic_with_delay(20e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(true)]);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::High, delay: 20e-9 }]);
    }

    #[test]
    fn a_low_result_requests_level_low_at_the_ics_delay() {
        let ic = ic_with_delay(35e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[Some(false)]);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::Low, delay: 35e-9 }]);
    }

    #[test]
    fn an_absent_result_resets_the_driver_instead_of_requesting_a_level() {
        let ic = ic_with_delay(20e-9);
        let d0 = DriverId(0);
        let reqs = driver_requests(&ic, &[d0], &[None]);
        assert_eq!(reqs, vec![DriverRequest::Reset { driver: d0 }]);
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
        let reqs = driver_requests(&ic, &[d0], &[Some(true)]);
        assert_eq!(reqs, vec![DriverRequest::Level { driver: d0, level: Mode::High, delay: 35e-9 }]);
    }

    #[test]
    fn maps_several_pins_in_order_matching_drivers_to_results_positionally() {
        let ic = ic_with_delay(0.0);
        let drivers = [DriverId(0), DriverId(1), DriverId(2), DriverId(3)];
        let result = [Some(true), None, Some(false), Some(true)];
        let reqs = driver_requests(&ic, &drivers, &result);
        assert_eq!(
            reqs,
            vec![
                DriverRequest::Level { driver: DriverId(0), level: Mode::High, delay: 0.0 },
                DriverRequest::Reset { driver: DriverId(1) },
                DriverRequest::Level { driver: DriverId(2), level: Mode::Low, delay: 0.0 },
                DriverRequest::Level { driver: DriverId(3), level: Mode::High, delay: 0.0 },
            ]
        );
    }
}
