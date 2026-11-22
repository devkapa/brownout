//! A load-time guard for a bench supply's current-limit rail the core reads
//! wrong (mechanism M3 of the current-limit diagnosis, ship-logs/ccd and
//! ship-logs/fixm3).
//!
//! A default-mode `bench_psu` hub whose rail carries a capacitor to a fixed
//! node, with a STEPPED hub partition and a CLOSED-FORM hub partition on the
//! rail, is held in current limit with the stepped partition's draw in its
//! frozen form: its `g` is a capacitor series conductance (`C / h`, 6.9e5 S on
//! the minimal board), so the segment aims the rail at `dl / g`, microvolts,
//! and the charge sent in goes nowhere. The rail reads up to volts off and a
//! capacitor current of millions of amps is published at a re-seed. The
//! proper fix (stepping the rail capacitors with the partition) is not
//! landed; until it is, such a board must not run on the core. Classic does it
//! right (ship-logs/guard3/NOTES.md).
//!
//! The test is the engine's own partition build: the supply is the hub rail,
//! the rail is not free (`NO_RAIL_SHORTCUT` frees it: exact mode steps the
//! rail with its partitions and reads these boards right, so the guard does not
//! apply there), the rail has a capacitor to a fixed node, and the build holds
//! a stepped hub partition and a closed-form hub partition. The stepped one
//! need not hold a capacitor from the rail: the narrower forms (a capacitor
//! from the rail into it, an esr-0 loop through the rail) miss boards the
//! diagnosis shows wrong, and the broad form catches none it shows right.

use super::CC_RAIL_REASON;
use crate::kinds;
use crate::partition::{PartitionBuild, PartitionKind};
use crate::program::Program;

/// The decline message when `program`'s build has the board described above,
/// else `None`.
pub(super) fn decline(program: &Program, pb: &PartitionBuild) -> Option<String> {
    let rail = pb.rail.as_ref().filter(|t| !t.exact && program.devices[t.device].kind == kinds::BENCH_PSU)?;
    let r = rail.rail;
    let fixed_to = |n: usize| n != r && pb.fixed[n].is_some();
    let rail_capacitor = program.devices.iter().any(|d| {
        d.kind == kinds::CAPACITOR && {
            let (a, b) = (d.pins[0].node, d.pins[1].node);
            (a == r && fixed_to(b)) || (b == r && fixed_to(a))
        }
    });
    if !rail_capacitor {
        return None;
    }
    let hub_of = |kind: PartitionKind| (0..pb.kinds.len()).any(|p| pb.hub[p] && pb.kinds[p] == kind);
    if !(hub_of(PartitionKind::Stepped) && hub_of(PartitionKind::ClosedForm)) {
        return None;
    }
    Some(format!(
        "{CC_RAIL_REASON}: bench_psu (device {}) has a rail capacitor (one from its rail to a fixed node) beside a stepped and a closed-form capacitor branch on that rail; the core does not yet read such a supply right in current limit (the rail can read volts off and a capacitor current of millions of amps), so the circuit runs on the classic engine, which does (exact rail mode, `noRailShortcut`, is not declined)",
        rail.device
    ))
}
