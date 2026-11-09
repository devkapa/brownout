//! W1g's half of [`PartitionedSolver`]: the settle, the slow events, the
//! reset, the records and the device state (core/README.md, "Kernel
//! semantics: thermal, failures and warnings (W1g)", points 2-17).
//!
//! `slow/*` holds the integrators and their journal and `digital::sag` the
//! `output_sag` window; this module decides when each runs, from what the
//! partitions published, and is the one place a slow change reaches a stamp
//! ([`PartitionedSolver::stamp_changed`]). Slow physics computes its own
//! currents from node voltages (point 1): nothing here reads `currents`,
//! `channels` or the source KCL.

use std::collections::HashMap;
use std::rc::Rc;

use super::stepped;
use super::static_part::{self, StampState};
use super::closed_form::DevicePin;
use super::{place_junction, DynJunction, KclDevice, PartitionedSolver};
use crate::devices::{self, led};
use crate::digital::sag::{self, Publish, SagCell, SagState};
use crate::digital::{self, power::IcPower};
use crate::drivers::Driver;
use crate::events::{DeviceId, EventKind, Mode, PartitionId};
use crate::kinds;
use crate::partition::{self, PartitionKind};
use crate::program::Program;
use crate::slow::failures::{self, FailureKind};
use crate::slow::stress::{PtcDrive, PtcParams, SegmentRate, StepMean, StressDrive};
use crate::slow::thermal::{PowerDrive, Shutdown, ThermalConfig};
use crate::slow::{is_due, warnings, ExpVoltage, LatchKey, LatchRecord, SlowEventKind};

/// Where a slow device's drive comes from between two publishes (point 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Class {
    /// A static partition: the pre-publish drive, held over the interval.
    Static,
    /// Every pin on a fixed node (no partition): as static, and published at
    /// load and when a live node moves.
    FixedPair,
    /// A closed-form partition (resistors, fuses and PTCs only): the
    /// segment's own exponential.
    ClosedForm,
    /// A stepped partition: the step's end drive, held over the step
    /// (classic's backward rule), checked at each committed step.
    Stepped,
}

/// One slow device (point 1), as loaded.
#[derive(Debug, Clone)]
pub(super) struct SlowDevice {
    pub device: usize,
    pub kind: u32,
    /// The record's params, without any slow flag.
    pub record: Vec<f64>,
    /// Its pins' nodes, in pin order.
    pub nodes: Vec<usize>,
    pub class: Class,
    /// Every partition it touches, ascending.
    pub parts: Vec<usize>,
    /// A resistor's Rating record (`pMax`, `rPower`); `(0, 0)` without one.
    pub p_max: f64,
    pub r_power: f64,
    /// Its index in `led_devices`, for an LED-family device.
    pub led: Option<usize>,
    /// Whether it has a code 91 record.
    pub thermal: bool,
    /// Its position in `kcl_devices` (a resistor, fuse or PTC).
    pub kcl: Option<usize>,
    /// Whether a closed-form or stepped partition stamps it, through
    /// `live_params`, rather than the static solver.
    pub dynamic_owner: bool,
    /// Ruling R38 (a): its stress can never grow at any voltage the circuit
    /// can put across it ([`voltage_span_bound`], [`never_stressed`],
    /// [`led_never_stressed`]), and it has no thermal identity. Such a device is never visited: its
    /// integrator stays at 0 under the idle drive, which every visit would
    /// leave as it is, so no publish notes it and it schedules nothing.
    pub cold: bool,
    /// Whether the static solver stamps any of it or folds it into its KCL (a
    /// static partition, or two fixed nodes): a stepped LED's `T_hold` lag
    /// then leaves that solver alone.
    pub static_stamps: bool,
}

/// One IC's `output_sag` cells (point 9): cell `first + c` is its
/// output-capable logic pin `outputs[c]`, driven by its `c`-th driver.
#[derive(Debug, Clone)]
pub(super) struct SagIc {
    pub device: usize,
    /// Its position in `PartitionedSolver::ics`.
    pub pos: usize,
    pub first: usize,
    pub outputs: Vec<usize>,
    /// Its power when both supplies are constant fixed nodes (neither free
    /// nor live): the same at every scan, so computed once at load.
    pub const_power: Option<IcPower>,
}

/// One `output_sag` cell's facts: its IC (index in `sag_ics`), its pin's
/// node, whether that node is in a closed-form partition (read on its
/// trajectory inside an instant), its driver's id (`u32::MAX` until the
/// driver table exists, [`PartitionedSolver::ensure_driver_cells`]), and its
/// IC's power when both supplies are constant: `power` is [`POWER_LIVE`]
/// when they are not, else whether it is powered, with its `v_gnd`, `vih`
/// and `vil` (NaN when not constant). One record, since a scan reads all of
/// it.
#[derive(Debug, Clone, Copy)]
pub(super) struct CellInfo {
    pub k: u32,
    pub node: u32,
    pub driver: u32,
    pub cf: bool,
    pub power: u8,
    pub v_gnd: f64,
    pub vih: f64,
    pub vil: f64,
}

const POWER_LIVE: u8 = 0;
const POWER_ON: u8 = 1;
const POWER_OFF: u8 = 2;

/// `sag::read(dir, v_pin, power).is_some()` for a powered IC, from a cell's
/// constant power: the same subtraction and comparison.
#[inline]
fn sags(info: &CellInfo, dir: sag::Dir, v_pin: f64) -> bool {
    let actual = v_pin - info.v_gnd;
    match dir {
        sag::Dir::High => actual < info.vih,
        sag::Dir::Low => actual > info.vil,
    }
}

/// What a cell's last observation left it as, for a settle that finds it
/// noted by static publishes alone ([`PartitionedSolver::observe_noted`]):
/// `GATE_ANY` observes it; `GATE_QUIET` (`Ok`, a driver that commands no
/// level) passes over it, since no voltage or power makes a pin that is no
/// target sag; `GATE_HIGH` and `GATE_LOW` (`Ok`, a powered target in that
/// direction on a constant supply, off a closed-form node) observe it only
/// when its pin now reads as sagging, by `sag::read`'s own comparison. Its
/// driver cannot have changed since without forcing the observation
/// ([`PartitionedSolver::note_driver_change`]), and a rollback or a reset
/// sends every cell back to `GATE_ANY`.
const GATE_ANY: u8 = 0;
const GATE_QUIET: u8 = 1;
const GATE_HIGH: u8 = 2;
const GATE_LOW: u8 = 3;

/// What a static publish of one node concerns (point 2).
///
/// `cells..cells_end` of `note_cells`: the cells whose output pin is on it,
/// then every cell of each IC with a supply on it. A static publish notes
/// exactly the cells whose own inputs moved: a cell's observation reads only
/// its pin, its driver and its IC's supplies, so one none of whose inputs
/// moved observes the same.
///
/// `leds..leds_end` of `led_on_node`: the static LEDs with safe voltages
/// (`led_safe`) a junction of which ends on it. Such an LED is noted node by
/// node, not with its partition: its visit reads nothing but its junction
/// voltages and its own state, so one none of whose nodes moved changes
/// nothing (and a quiet one, [`led_quiet`], returns at once).
///
/// `part`: its partition when `part_noted` holds for it, else `u32::MAX`.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct NodeNote {
    pub cells: u32,
    pub cells_end: u32,
    pub leds: u32,
    pub leds_end: u32,
    pub part: u32,
}

/// A slow LED's range of `safe_junctions` (empty for any other device),
/// with its device index.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct SafeLed {
    pub device: u32,
    pub start: u32,
    pub end: u32,
}

impl SafeLed {
    pub fn is_empty(&self) -> bool {
        self.start == self.end
    }
}

/// One junction of a safe LED, in `led_devices` order: its nodes and the
/// largest voltage across them at which it carries at most `ifMax`.
#[derive(Debug, Clone, Copy)]
pub(super) struct SafeJunction {
    pub anode: u32,
    pub cathode: u32,
    pub v_safe: f64,
}

/// The load-time tables the settle reads.
#[derive(Debug, Default)]
pub(super) struct SlowModel {
    pub devs: Vec<SlowDevice>,
    /// device index -> its index in `devs`, `u32::MAX` for none.
    pub of_device: Vec<u32>,
    /// partition -> the slow devices touching it, except the static LEDs
    /// noted node by node ([`NodeNote`]).
    pub part_devs: Vec<Vec<u32>>,
    /// The fixed-pair and static devices with a pin or junction on a live
    /// node, visited at every settle (a rail move re-drives them whether or
    /// not a node of their partition moved).
    pub live_devs: Vec<u32>,
    pub sag_ics: Vec<SagIc>,
    /// device index -> its index in `sag_ics`, `u32::MAX` for none.
    pub sag_of_device: Vec<u32>,
    /// Every `output_sag` cell (the index `SlowState`'s sag table uses).
    pub cells: Vec<CellInfo>,
    /// partition -> every cell whose output pin is free in it, and the ICs
    /// whose supply is free in it (all of whose cells a publish there moves).
    pub part_cells: Vec<Vec<u32>>,
    pub part_sag: Vec<Vec<u32>>,
    /// Per node, what a static publish moving it concerns ([`NodeNote`]),
    /// as ranges of `note_cells` and `led_on_node`: a node publishes tens of
    /// thousands of times a second, so one record holds all of it.
    pub node_note: Vec<NodeNote>,
    pub note_cells: Vec<u32>,
    pub led_on_node: Vec<u32>,
    /// partition -> whether a publish of it concerns the settle through its
    /// partition (slow devices touching it; for a dynamic partition, its
    /// cells and supplies too, which no changed-node list reports).
    pub part_noted: Vec<bool>,
    /// The ICs with a supply or output on a live fixed node, scanned at every
    /// settle (a rail move publishes no partition of theirs).
    pub live_sag: Vec<u32>,
    /// Each stepped partition's junctions as placed at load, with their
    /// devices: the list its shape is rebuilt from (`SteppedSolver::set_junctions`).
    pub stepped_junctions: Vec<(usize, Vec<(usize, DynJunction)>)>,
    /// Per `PartitionedSolver::junctions` entry, its law as loaded.
    pub load_laws: Vec<led::JunctionLaw>,
    /// Per slow LED device without a thermal identity, the largest junction
    /// voltage at which each of its junctions (in `led_devices` order) carries
    /// at most `ifMax`: at or below them `peakI <= ifMax`, so the damage rate
    /// is exactly 0 and the visit needs no exponential (the SAP-1's 83 LEDs
    /// sit far below their limit). An empty range for every other device.
    /// Flat, with the junctions' nodes, since a visit reads them first.
    pub led_safe: Vec<SafeLed>,
    pub safe_junctions: Vec<SafeJunction>,
    /// Whether there is anything to settle at all.
    pub any: bool,
}

/// What happened since the last settle, and the settle's own working lists.
#[derive(Debug, Default)]
pub(super) struct SlowScratch {
    /// Ruling R71 (b): the stepped devices the trail closed at this instant,
    /// by slow index; their visit keeps the drive the trail left.
    pub trailed: Vec<u32>,
    /// The partitions published since the last settle (point 2).
    pub pub_flag: Vec<bool>,
    pub pub_list: Vec<u32>,
    /// Drivers the kernel changed (a commit can flip a commanded direction
    /// without moving a node).
    pub driver_changes: Vec<u32>,
    /// driver id -> its `output_sag` cell (`u32::MAX` for none); derived
    /// once the driver table exists.
    pub driver_cell: Vec<u32>,
    /// The cells to observe at the next settle, one bit each, and whether
    /// any is set: a set is cheaper to note into and to drain than a list,
    /// and it drains in cell order, each cell once. `cell_bits` holds the
    /// static publishes' notes, which a cell's gate may pass over;
    /// `cell_force` the rest (a driver change, a dynamic publish, a window,
    /// a live node, a load or a reset), which always observe.
    pub cell_bits: Vec<u64>,
    pub cell_force: Vec<u64>,
    pub cells_any: bool,
    /// Each cell's gate ([`GATE_ANY`] and the rest).
    pub cell_gate: Vec<u8>,
    /// The slow devices to visit at the next settle, and which of them to
    /// force, as bits: drained in device order, each once.
    pub dev_bits: Vec<u64>,
    pub force_bits: Vec<u64>,
    pub devs_any: bool,
    /// The slow journal is opened at an instant's first slow write, not at
    /// every instant (0 closed, 1 window open but journal not begun, 2 begun):
    /// most instants of a digital board write no slow state at all.
    pub journal: u8,
    /// ICs whose window event fired (point 9: the event only marks it due).
    pub sag_due: Vec<u32>,
    /// Slow devices to re-drive and reschedule whatever their drive.
    pub revisit: Vec<u32>,
    /// Scan every IC at the next settle (load, reset).
    pub sag_all: bool,
    /// Whether the first settle (load) has run.
    pub loaded: bool,
    /// Partitions a slow change dirtied, for the caller.
    pub dirty: Vec<PartitionId>,
    /// A device with no partition changed its stamp: its current and the
    /// source KCL are republished at that instant (point 12).
    pub republish: bool,
    /// Devices whose stamps were set in the open window, put back after a
    /// rollback.
    pub touched: Vec<usize>,
    /// Each PTC's trip time (channel 2's `since`), with the window's undo log.
    pub ptc_since: Vec<f64>,
    pub ptc_log: Vec<(u32, f64)>,
    /// The CC rail capacitor's segment as the last settle saw it (R30): a
    /// new one re-drives every static and fixed-pair device the rail reaches,
    /// whose drives follow it. `None` (not yet seen) at load and after a
    /// rollback, which re-drives them once more.
    pub rail_seg: Option<Option<crate::rails::CapSeg>>,
}

/// One device's drives at an instant.
#[derive(Default)]
struct Drives {
    stress: Option<StressDrive>,
    power: Option<PowerDrive>,
    ptc: Option<PtcDrive>,
    /// When a rail-driven segment's anchor must be renewed (ruling R21);
    /// infinity (or the default 0, never read) otherwise.
    redrive_by: f64,
}

/// A rail-driven segment's drive is re-anchored before the rail terms it
/// drops can move a pin by more than this plus [`DRIVE_TOL_REL`] of its
/// voltage: a tenth of W0's volts tolerance (100 uV plus 1e-3), shared by the
/// two pins.
const DRIVE_TOL_V: f64 = 1e-5;
const DRIVE_TOL_REL: f64 = 1e-4;

/// A device's own voltage is re-anchored (R29) before what the anchor drops
/// can move it by more than this plus [`DEVICE_TOL_REL`] of it: the
/// relative part is a tenth of W0's, and the absolute part is small enough
/// for a device whose whole voltage is a millivolt-scale lag behind a
/// moving rail.
const DEVICE_TOL_V: f64 = 1e-8;
const DEVICE_TOL_REL: f64 = 1e-4;

/// The least spacing of a device's re-anchors: twice the no-progress window,
/// so a device's own re-anchors never make a run of instants by themselves.
const REDRIVE_MIN_DT: f64 = 2.0 * crate::kernel::NO_PROGRESS_DT;

/// A node voltage as a closed form: a fixed value, or a segment's
/// exponential.
enum NodeV {
    Fixed(f64),
    Exp(ExpVoltage),
}

/// Whether two PTC drives integrate the same from `now` on: a constant cool
/// drive that has held since before `now` is not restarted by a publish that
/// finds it still cool.
fn ptc_same(old: PtcDrive, new: PtcDrive, now: f64) -> bool {
    match (old, new) {
        (PtcDrive::Stress(a), PtcDrive::Stress(b)) => a == b,
        (PtcDrive::Cool { from: f1, until: u1 }, PtcDrive::Cool { from: f2, until: u2 }) => {
            (u1 == f64::INFINITY && u2 == f64::INFINITY && f1 <= now && f2 <= now) || (f1.to_bits() == f2.to_bits() && u1.to_bits() == u2.to_bits())
        }
        _ => false,
    }
}

/// The largest junction voltage at which `law` carries at most `limit`
/// (bisection on the monotone law, keeping the side that does), or infinity
/// when no voltage the law can carry exceeds it (`limit <= 0` means no
/// limit, as the damage rate reads it). A reverse current is at most `Is` in
/// magnitude, so a junction under this voltage has `|I| <= limit` whenever
/// `Is <= limit`; otherwise nothing is safe and the visit always evaluates.
pub(super) fn safe_voltage(law: &led::JunctionLaw, limit: f64) -> f64 {
    if limit.is_nan() || limit <= 0.0 {
        return f64::INFINITY;
    }
    if law.is.is_nan() || law.is > limit {
        return f64::NEG_INFINITY;
    }
    let (mut lo, mut hi) = (0.0, law.vsat.max(1.0));
    while law.current(hi) <= limit {
        hi *= 2.0;
        if !hi.is_finite() {
            return f64::INFINITY;
        }
    }
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if law.current(mid) <= limit {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

/// Ruling R38 (a): a cold device's rate is tested at the span bound widened
/// by this much, which covers how far a solver's published voltage can stray
/// past the exact one the bound holds for (W0's 100 uV plus 1e-3, the
/// backward-Euler controller's step error). It is a tolerance on a sound
/// bound, not a cover for an unsound one: an overshoot that grows with the
/// circuit (a trapezoidal ring, a wrong rail solve) has no bound at all.
pub(super) const COLD_MARGIN_REL: f64 = 1e-2;
pub(super) const COLD_MARGIN_V: f64 = 1e-2;

/// Test-only switch that loads every slow device hot, so a test can run a
/// circuit's visited twin and compare (R38 (a): a cold device's record must
/// be its visited twin's, empty). Per thread, armed by a guard, as
/// `newton::hooks`.
#[cfg(test)]
pub(super) mod cold_hooks {
    use std::cell::Cell;

    thread_local! {
        static OFF: Cell<bool> = const { Cell::new(false) };
    }

    /// Re-enables cold devices on drop.
    pub struct Guard;

    impl Drop for Guard {
        fn drop(&mut self) {
            OFF.with(|c| c.set(false));
        }
    }

    /// Every load from now until the guard drops makes no device cold.
    pub fn no_cold() -> Guard {
        OFF.with(|c| c.set(true));
        Guard
    }

    pub(crate) fn disabled() -> bool {
        OFF.with(|c| c.get())
    }
}

#[cfg(not(test))]
mod cold_hooks {
    #[inline(always)]
    pub(super) fn disabled() -> bool {
        false
    }
}

/// Ruling R38 (a): an upper bound on the voltage across any two nodes of one
/// section at any time, `hi - lo` over the potentials every fixed node and
/// the rail can take, with `max(|lo|, |hi|)`, the most any node's own
/// potential can be; `None` when the program holds something the bound does
/// not cover. It is constant from the load on: no source has a live param,
/// so nothing after the load can raise it.
///
/// - **Fixed nodes and the rail.** Walking the propagation forest
///   (`resolved`) from node 0, each child's range is its parent's plus its
///   source's: a `voltage_source` or an ideal `bench_psu` its value, a clock
///   or pulse its two levels, a section anchor 0, a battery `[0, max E]`
///   over its OCV table (`E(soc)` never leaves the table's knots, and a load
///   only pulls the terminal below `E`).
/// - **No current-limited `bench_psu`.** Its rail is a solved node, not a
///   held one, and nothing short of a correct rail solve keeps it in `[0,
///   V]`: a supply that cannot sink lets another source lift it past `V`,
///   and a wrong solve has published it far outside (W1gV4: 209.5 V on an
///   11.29 V supply after a shorting switch opens, 9.901 V on a 5 V one
///   back-driven through 10 ohm). A bound read off the rail's own solve is
///   only as sound as that solve, so there is none.
/// - **No trapezoidal stepping** (`may_trap`). `INTEGRATION` 1 steps every
///   stepped partition by the trapezoidal rule, and R79 (a) a linear one off
///   the hub under either option; it has no maximum principle: past
///   a stiff edge it rings, and a capacitor charged through an LED lands
///   volts past its supply and stays there while it discharges (W1gV4: 3.63
///   V over a 12.31 V supply for 0.3 s). Backward Euler, the closed forms
///   and the static solves keep every free node inside; a trapezoidal
///   partition does not, and a node it overshoots can be an IC's supply
///   that another partition's outputs follow, so no partition gets a bound.
/// - **Free nodes** stay within `[lo, hi]` (the maximum principle). Every
///   element a free node sees is downhill: a resistance (resistor, fuse, PTC,
///   switch contact, an IC output's `R_out` to its own `vcc` or `gnd` node, a
///   555's output and discharge, the shunts to node 0), a junction (its
///   current has the sign of its voltage), or a floating pull to a target that
///   is its IC's `vcc` or `gnd` potential. At the highest free node every
///   such element carries current out, so it cannot rise above every fixed
///   potential by itself: a node without a capacitor would break KCL, and a
///   capacitor whose other pin is on a constant fixed node discharges.
///   Hence two conditions, or no bound:
///   - every kind is one of the above (a later kind is not covered until it
///     is added here);
///   - every capacitor with a free pin has its other pin on a fixed node that
///     never moves: a capacitor between two free nodes, or from a free node to
///     a moving one, can pump a node past every source (a charge pump).
pub(super) fn voltage_span_bound(program: &Program, resolved: &[partition::ResolvedSource], fixed: &[Option<f64>], live: &[bool], may_trap: bool) -> Option<(f64, f64)> {
    if may_trap {
        return None;
    }
    for d in &program.devices {
        let known = digital::is_ic(d.kind)
            || matches!(
                d.kind,
                kinds::RESISTOR
                    | kinds::VOLTAGE_SOURCE
                    | kinds::CAPACITOR
                    | kinds::SWITCH
                    | kinds::PUSH_BUTTON
                    | kinds::SPDT_SWITCH
                    | kinds::DIP_SWITCH
                    | kinds::FUSE
                    | kinds::PTC_FUSE
                    | kinds::BATTERY_PACK
                    | kinds::BENCH_PSU
                    | kinds::CLOCK
                    | kinds::CLOCK_GEN
                    | kinds::PULSE_SOURCE
                    | kinds::PULSE_GEN
                    | kinds::SECTION_ANCHOR
                    | kinds::LED
                    | kinds::RGB_LED
                    | kinds::BICOLOR_LED
                    | kinds::SEG7_CC
                    | kinds::SEG7_CA
                    | kinds::RATING
                    | kinds::THERMAL
            );
        if !known {
            return None;
        }
        if d.kind == kinds::CAPACITOR {
            let (u, w) = (d.pins[0].node, d.pins[1].node);
            let constant = |n: usize| fixed[n].is_some() && !live[n];
            let pumps = match (fixed[u].is_none(), fixed[w].is_none()) {
                (true, true) => u != w,
                (true, false) => !constant(w),
                (false, true) => !constant(u),
                (false, false) => false,
            };
            if pumps {
                return None;
            }
        }
    }
    let mut range: Vec<Option<(f64, f64)>> = vec![None; fixed.len()];
    range[0] = Some((0.0, 0.0));
    for r in resolved {
        let d = &program.devices[r.device_index];
        let p = &d.params;
        let span_of = |a: f64, b: f64| if a <= b { (a, b) } else { (b, a) };
        let (lo, hi) = match d.kind {
            kinds::SECTION_ANCHOR => (0.0, 0.0),
            kinds::VOLTAGE_SOURCE => (p[0], p[0]),
            kinds::BENCH_PSU if !(p[1] > 0.0) => (p[0], p[0]),
            // A current-limited one's rail falls through to `None` (above).
            kinds::CLOCK | kinds::CLOCK_GEN => span_of(p[2], p[3]),
            kinds::PULSE_SOURCE | kinds::PULSE_GEN => span_of(p[0], p[1]),
            kinds::BATTERY_PACK => {
                let (a, b) = devices::battery::BatteryModel::from_record(p, &d.blob).emf_range()?;
                (a.min(0.0), b.max(0.0))
            }
            _ => return None,
        };
        let (plo, phi) = range[r.parent_node]?;
        range[r.child_node] = Some(if r.parent_is_pos { (plo - hi, phi - lo) } else { (plo + lo, phi + hi) });
    }
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for (n, f) in fixed.iter().enumerate() {
        if f.is_some() && range[n].is_none() {
            return None;
        }
    }
    for &(a, b) in range.iter().flatten() {
        if !(a.is_finite() && b.is_finite()) {
            return None;
        }
        lo = lo.min(a);
        hi = hi.max(b);
    }
    Some((hi - lo, lo.abs().max(hi.abs())))
}

/// A two-terminal resistance's least resistance, `None` for any other kind:
/// a resistor's stamped `R`, a fuse's or PTC's `rNormal` (a latch opens a
/// fuse, a trip multiplies a PTC's by 200, and either only lowers the
/// current through it).
fn least_resistance(device: &crate::program::Device) -> Option<f64> {
    let r = match device.kind {
        kinds::RESISTOR => device.params[0],
        kinds::FUSE => device.params[kinds::fuse_param::R_NORMAL],
        kinds::PTC_FUSE => device.params[kinds::ptc_param::R_NORMAL],
        _ => return None,
    };
    (r > 0.0 && r.is_finite()).then_some(r)
}

/// Ruling R38 (a): the most voltage two-terminal resistance `d` can carry,
/// given [`voltage_span_bound`]'s `(span, v_abs)` already widened.
///
/// Across the device it is at most the span. When the device is in series
/// with other resistances through free nodes that nothing else touches (each
/// such node has exactly two pins on it, of two two-terminal resistances),
/// KCL makes the chain one current, less what each node's shunt to node 0
/// leaks (`NODE_RSHUNT_G * v_abs` at most). The chain's two ends are nodes,
/// so at most the span lies across all of it, and `|I| <= span/R_total + m
/// G v_abs` over its `m` inner nodes: the device's share is `R_d` times that.
/// A 555's 1 mOhm timing resistor in series with 1 kOhm then carries
/// microvolts, not the supply. Any member's least resistance stands in for
/// it (a latch or a trip only raises a resistance), and the walk may stop at
/// any node: a shorter chain bounds less tightly, never wrongly.
pub(super) fn resistance_volts_bound(program: &Program, d: usize, pins_on: &[u32], devs_on: &[Vec<u32>], fixed: &[Option<f64>], span: f64, v_abs: f64) -> f64 {
    let Some(r_d) = least_resistance(&program.devices[d]) else { return span };
    let (r_others, inner) = series_chain(program, d, pins_on, devs_on, fixed);
    let bound = r_d * (span / (r_d + r_others) + inner as f64 * static_part::NODE_RSHUNT_G * v_abs);
    if bound < span {
        bound
    } else {
        span
    }
}

/// The resistances in series with two-terminal device `d` through free
/// nodes nothing else touches ([`resistance_volts_bound`]): their least
/// resistances summed, and how many such nodes the walk crossed.
fn series_chain(program: &Program, d: usize, pins_on: &[u32], devs_on: &[Vec<u32>], fixed: &[Option<f64>]) -> (f64, usize) {
    let device = &program.devices[d];
    let (mut r_total, mut inner) = (0.0, 0usize);
    if device.pins.len() != 2 {
        return (r_total, inner);
    }
    for start in 0..2 {
        let (mut node, mut from) = (device.pins[start].node, d);
        let mut steps = 0;
        while fixed[node].is_none() && pins_on[node] == 2 && steps < program.devices.len() {
            let Some(&next) = devs_on[node].iter().find(|&&e| e as usize != from) else { break };
            let next = next as usize;
            let e = &program.devices[next];
            let Some(r_e) = least_resistance(e) else { break };
            if next == d || e.pins.len() != 2 || e.pins[0].node == e.pins[1].node {
                break;
            }
            r_total += r_e;
            inner += 1;
            node = if e.pins[0].node == node { e.pins[1].node } else { e.pins[0].node };
            from = next;
            steps += 1;
        }
    }
    (r_total, inner)
}

/// Ruling R38 (a): whether a plain `led` (one junction, no thermal identity)
/// can never carry more than `if_max` forward, so that its damage rate
/// `max(0, (peakI/ifMax)^2 - 1)` is 0 for good. Over `if_max` its junction is
/// past `v_safe` (the law is monotone) and the resistances in series with it
/// ([`series_chain`]) each carry at least `if_max` less the node shunts'
/// leak, so the chain would hold more than `v_safe + r_others (if_max - m G
/// v_abs)`, which cannot exceed the span (`span`, `v_abs` already widened).
/// A reverse current is at most `Is`, which `safe_voltage` holds to `if_max`;
/// with no limit (`if_max <= 0`) the rate is 0 at any current.
pub(super) fn led_never_stressed(v_safe: f64, if_max: f64, r_others: f64, inner: usize, span: f64, v_abs: f64) -> bool {
    if v_safe == f64::INFINITY {
        return true;
    }
    v_safe.is_finite() && if_max > 0.0 && v_safe + r_others * (if_max - inner as f64 * static_part::NODE_RSHUNT_G * v_abs) > span
}

/// Ruling R38 (a): whether a slow device's stress can never grow while no
/// voltage across it exceeds `v` (already widened), tested with the visit's
/// own rate in both forms (the constant drive's and the closed-form
/// segment's `k V^2 - c`), each monotone in `|V|`. A device with a
/// thermal identity is never cold: its readout reports the interval's power,
/// held to `1e-9` W, which only a visit keeps. PTCs (trip and recovery) and
/// LEDs (their visits are already cheap below their safe voltages) stay as
/// they are.
pub(super) fn never_stressed(kind: u32, record: &[f64], p_max: f64, r_power: f64, thermal: bool, v: f64) -> bool {
    if thermal {
        return false;
    }
    match kind {
        kinds::RESISTOR => {
            let r = record[0];
            let k = if r_power > 0.0 { r_power / (r * r * p_max) } else { 0.0 };
            p_max > 0.0 && failures::resistor_overload(v, r, r_power, p_max).rate <= 0.0 && k * v * v - 1.0 <= 0.0
        }
        kinds::FUSE => {
            use kinds::fuse_param::*;
            let (r_normal, i_rating) = (record[R_NORMAL], record[I_RATING]);
            let k = if i_rating > 0.0 { 1.0 / ((r_normal * i_rating) * (r_normal * i_rating)) } else { 0.0 };
            failures::fuse_tripped(v, r_normal, i_rating).rate <= 0.0 && k * v * v - 1.0 <= 0.0
        }
        _ => false,
    }
}

/// Whether a visit of device `i` would change nothing, decided without the
/// exponential: an LED-family device without a thermal identity that has
/// failed (it integrates nothing more), or that holds no stress under a drive
/// that cannot accumulate and sits at or under its safe voltages, where the
/// full evaluation gives the same idle drive, writes nothing and has no event
/// (`led_safe`).
fn led_quiet(m: &SlowModel, slow: &crate::slow::SlowState, voltages: &[f64], i: usize) -> bool {
    let safe = m.led_safe[i];
    if safe.is_empty() {
        return false;
    }
    let dev = safe.device as usize;
    if slow.is_failed(dev) {
        return true;
    }
    led_below_safe(m, voltages, i) && slow.stress(dev).is_none_or(|s| s.s0 == 0.0 && !s.drive.accumulates())
}

/// Every junction of LED device `i` at or under its safe voltage.
fn led_below_safe(m: &SlowModel, voltages: &[f64], i: usize) -> bool {
    let safe = m.led_safe[i];
    !safe.is_empty() && m.safe_junctions[safe.start as usize..safe.end as usize].iter().all(|j| voltages[j.anode as usize] - voltages[j.cathode as usize] <= j.v_safe)
}

/// What a static publish of `node` concerns (point 2): the cells on it,
/// every cell of an IC supplied from it, and its partition when slow
/// devices touch it. Called for each node whose published bits moved.
pub(super) fn note_static_node(s: &mut SlowScratch, m: &SlowModel, node: usize) {
    let n = m.node_note[node];
    let (a, b) = (n.cells as usize, n.cells_end as usize);
    if a < b {
        for &cell in &m.note_cells[a..b] {
            s.cell_bits[(cell >> 6) as usize] |= 1 << (cell & 63);
        }
        s.cells_any = true;
    }
    let (a, b) = (n.leds as usize, n.leds_end as usize);
    if a < b {
        for &i in &m.led_on_node[a..b] {
            s.dev_bits[(i >> 6) as usize] |= 1 << (i & 63);
        }
        s.devs_any = true;
    }
    let p = n.part;
    if p != u32::MAX && !s.pub_flag[p as usize] {
        s.pub_flag[p as usize] = true;
        s.pub_list.push(p);
    }
}


impl PartitionedSolver {
    // ---- load ----------------------------------------------------------

    /// Builds the slow model from the program (points 1, 3-10): the slow
    /// devices with their integrators, the `output_sag` cells, and what the
    /// stamp refresh needs. A load starts every slow device fresh (point 15).
    pub(super) fn wire_w1g(&mut self, program: &Program, live: &[bool], fixed: &[Option<f64>]) {
        let device_count = program.devices.len();
        let part_count = self.partition_kinds.len();
        let start = program.options.start_time;
        let ambient = program.options.ambient_c;
        let mut model = SlowModel { of_device: vec![u32::MAX; device_count], sag_of_device: vec![u32::MAX; device_count], part_devs: vec![Vec::new(); part_count], part_sag: vec![Vec::new(); part_count], part_cells: vec![Vec::new(); part_count], ..SlowModel::default() };
        let mut node_cells: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];
        let mut node_supply: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];

        let mut rating: Vec<Option<(f64, f64)>> = vec![None; device_count];
        let mut thermal: Vec<Option<ThermalConfig>> = vec![None; device_count];
        for d in &program.devices {
            match d.kind {
                kinds::RATING => {
                    use kinds::rating_param::*;
                    rating[d.params[DEVICE] as usize] = Some((d.params[P_MAX], d.params[R_POWER]));
                }
                kinds::THERMAL => {
                    use kinds::thermal_param::*;
                    let shutdown = (d.params[HAS_SHUTDOWN] != 0.0).then(|| Shutdown { trip_c: d.params[TRIP_C], restart_c: d.params[RESTART_C] });
                    thermal[d.params[DEVICE] as usize] = Some(ThermalConfig { ambient_c: ambient, r_theta: d.params[R_THETA], tau_s: d.params[TAU_S], shutdown, holds_junction: false });
                }
                _ => {}
            }
        }

        // R38 (a): the bound every cold test reads, widened, with what the
        // series walk needs (the pins on each node and their devices).
        let may_trap = self.stepped.may_step_trap(stepped::Method::from_option(program.options.integration));
        let span = voltage_span_bound(program, &self.resolved, fixed, live, may_trap).filter(|_| !cold_hooks::disabled()).map(|(span, v_abs)| (span * (1.0 + COLD_MARGIN_REL) + COLD_MARGIN_V, v_abs * (1.0 + COLD_MARGIN_REL) + COLD_MARGIN_V));
        let mut pins_on = vec![0u32; program.node_count];
        let mut devs_on: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];
        if span.is_some() {
            for (e, device) in program.devices.iter().enumerate() {
                for p in &device.pins {
                    pins_on[p.node] += 1;
                    if devs_on[p.node].last() != Some(&(e as u32)) {
                        devs_on[p.node].push(e as u32);
                    }
                }
            }
        }
        let led_of: HashMap<usize, usize> = self.led_devices.iter().enumerate().map(|(e, l)| (l.device_index, e)).collect();
        let part_of = |node: usize| self.local_index[node].map(|(p, _)| p);
        for (d, device) in program.devices.iter().enumerate() {
            let (p_max, r_power) = rating[d].unwrap_or((0.0, 0.0));
            let is_slow = match device.kind {
                kinds::RESISTOR => p_max > 0.0 || thermal[d].is_some(),
                kinds::FUSE | kinds::PTC_FUSE | kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED => true,
                _ => false,
            };
            if !is_slow {
                continue;
            }
            let nodes: Vec<usize> = device.pins.iter().map(|p| p.node).collect();
            let mut parts: Vec<usize> = match device.kind {
                kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED => {
                    led::junctions(d, device, ambient).iter().filter_map(|j| super::junction_partition(j, &self.local_index)).collect()
                }
                _ => nodes.iter().filter_map(|&n| part_of(n)).collect(),
            };
            parts.sort_unstable();
            parts.dedup();
            let kinds_of: Vec<PartitionKind> = parts.iter().map(|&p| self.partition_kinds[p]).collect();
            let class = if kinds_of.contains(&PartitionKind::Stepped) {
                Class::Stepped
            } else if kinds_of.contains(&PartitionKind::ClosedForm) {
                Class::ClosedForm
            } else if kinds_of.is_empty() {
                Class::FixedPair
            } else {
                Class::Static
            };
            let static_stamps = match device.kind {
                kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED => led::junctions(d, device, ambient)
                    .iter()
                    .filter(|j| j.stamped())
                    .any(|j| super::junction_partition(j, &self.local_index).is_none_or(|p| self.partition_kinds[p] == PartitionKind::Static)),
                _ => !matches!(class, Class::ClosedForm | Class::Stepped),
            };
            let i = model.devs.len() as u32;
            let threshold = match device.kind {
                kinds::RESISTOR => (p_max > 0.0).then_some(failures::RESISTOR_DAMAGE_THRESHOLD),
                kinds::FUSE => Some(failures::FUSE_I2T_THRESHOLD),
                kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED => Some(failures::LED_DAMAGE_THRESHOLD),
                _ => None,
            };
            if let Some(threshold) = threshold {
                self.slow.init_stress(d, threshold, start);
            }
            if device.kind == kinds::PTC_FUSE {
                use kinds::ptc_param::*;
                self.slow.init_ptc(d, PtcParams { r_normal: device.params[R_NORMAL], i_hold: device.params[I_HOLD] }, start);
            }
            if let Some(mut cfg) = thermal[d] {
                // A plain `led` follows its held temperature; in a stepped
                // partition by the lag rule, elsewhere at a band boundary
                // (point 4).
                cfg.holds_junction = device.kind == kinds::LED && class != Class::Stepped;
                self.slow.init_thermal(d, cfg, start);
            }
            let cold = span.is_some_and(|(span, v_abs)| match device.kind {
                kinds::LED => {
                    let junctions = led::junctions(d, device, ambient);
                    match junctions.as_slice() {
                        [j] if thermal[d].is_none() && j.stamped() => {
                            let if_max = led::if_max(device).unwrap_or(0.0);
                            let (r_others, inner) = series_chain(program, d, &pins_on, &devs_on, fixed);
                            led_never_stressed(safe_voltage(&j.law, if_max), if_max, r_others, inner, span, v_abs)
                        }
                        _ => false,
                    }
                }
                _ => {
                    let v = resistance_volts_bound(program, d, &pins_on, &devs_on, fixed, span, v_abs);
                    never_stressed(device.kind, &device.params, p_max, r_power, thermal[d].is_some(), v)
                }
            });
            if !cold {
                for &p in &parts {
                    model.part_devs[p].push(i);
                }
                if class == Class::FixedPair && nodes.iter().any(|&n| live[n]) {
                    model.live_devs.push(i);
                }
            }
            model.of_device[d] = i;
            model.devs.push(SlowDevice {
                device: d,
                kind: device.kind,
                record: device.params.clone(),
                nodes,
                class,
                parts,
                p_max,
                r_power,
                led: led_of.get(&d).copied(),
                thermal: thermal[d].is_some(),
                kcl: self.kcl_position.get(&d).copied(),
                dynamic_owner: matches!(class, Class::ClosedForm | Class::Stepped),
                static_stamps,
                cold,
            });
        }

        for sd in &model.devs {
            let start = model.safe_junctions.len() as u32;
            if let (Some(e), false) = (sd.led, sd.thermal) {
                let e = &self.led_devices[e];
                let limit = led::if_max(&e.device).unwrap_or(0.0);
                model.safe_junctions.extend(e.junctions.iter().map(|j| SafeJunction { anode: j.anode as u32, cathode: j.cathode as u32, v_safe: safe_voltage(&j.law, limit) }));
            }
            model.led_safe.push(SafeLed { device: sd.device as u32, start, end: model.safe_junctions.len() as u32 });
        }

        // The `output_sag` cells: every output-capable pin of every IC.
        let mut cells = Vec::new();
        for (pos, (d, ic)) in self.ics.iter().enumerate() {
            let outputs: Vec<usize> = digital::output_capable(ic.kind).to_vec();
            let k = model.sag_ics.len() as u32;
            let first = cells.len();
            cells.extend(outputs.iter().map(|&li| SagCell { device: *d, pin: li + 2, state: SagState::Ok }));
            let touched = [ic.vcc.node, ic.gnd.node].into_iter().chain(outputs.iter().map(|&li| ic.logic[li].node));
            if touched.into_iter().any(|n| self.local_index[n].is_none() && live[n]) {
                model.live_sag.push(k);
            }
            let mut supply_parts: Vec<usize> = [ic.vcc.node, ic.gnd.node].into_iter().filter_map(part_of).collect();
            supply_parts.sort_unstable();
            supply_parts.dedup();
            for p in supply_parts {
                model.part_sag[p].push(k);
            }
            for n in [ic.vcc.node, ic.gnd.node] {
                if !node_supply[n].contains(&k) {
                    node_supply[n].push(k);
                }
            }
            let constant = |n: usize| self.local_index[n].is_none() && !live[n];
            // The constant power below is the free verdict: tables are built
            // at load, before any hold exists.
            debug_assert_eq!(self.holds_active, 0, "no power hold exists at load");
            let const_power = (constant(ic.vcc.node) && constant(ic.gnd.node)).then(|| digital::power::ic_power(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]));
            let (power, v_gnd, vih, vil) = match const_power {
                Some(p) => (if p.powered { POWER_ON } else { POWER_OFF }, p.v_gnd, p.vih, p.vil),
                None => (POWER_LIVE, f64::NAN, f64::NAN, f64::NAN),
            };
            for &li in &outputs {
                let node = ic.logic[li].node;
                let cell = model.cells.len() as u32;
                let cf = part_of(node).is_some_and(|p| self.partition_kinds[p] == PartitionKind::ClosedForm);
                model.cells.push(CellInfo { k, node: node as u32, driver: u32::MAX, cf, power, v_gnd, vih, vil });
                node_cells[node].push(cell);
                if let Some(p) = part_of(node) {
                    model.part_cells[p].push(cell);
                }
            }
            model.sag_of_device[*d] = k;
            model.sag_ics.push(SagIc { device: *d, pos, first, outputs, const_power });
        }
        self.slow.init_sag(cells);

        for &p in &self.stepped_parts {
            let placed: Vec<(usize, DynJunction)> = partition::program_junctions(program)
                .iter()
                .filter_map(|j| place_junction(j, p, &self.local_index, fixed).map(|dj| (j.device, dj)))
                .collect();
            if !placed.is_empty() {
                model.stepped_junctions.push((p, placed));
            }
        }
        // A static device with a pin or junction on a live fixed node is
        // visited at every settle, as a fixed pair on one is: a rail move
        // re-drives it whether or not a node of its partition moved bits. A
        // static LED with safe voltages and no live node is noted node by
        // node instead of with its partitions (`led_on_node`).
        let mut led_nodes: Vec<Vec<u32>> = vec![Vec::new(); program.node_count];
        for i in 0..model.devs.len() {
            let sd = &model.devs[i];
            if sd.class != Class::Static || sd.cold {
                continue;
            }
            let mut nodes = sd.nodes.clone();
            if let Some(e) = sd.led {
                for j in &self.led_devices[e].junctions {
                    nodes.extend([j.anode, j.cathode]);
                }
            }
            if nodes.iter().any(|&n| live[n]) {
                model.live_devs.push(i as u32);
                continue;
            }
            if model.led_safe[i].is_empty() {
                continue;
            }
            nodes.sort_unstable();
            nodes.dedup();
            for &n in &nodes {
                led_nodes[n].push(i as u32);
            }
            for &p in &sd.parts {
                model.part_devs[p].retain(|&d| d != i as u32);
            }
        }
        model.live_devs.sort_unstable();
        model.part_noted = (0..part_count)
            .map(|p| !model.part_devs[p].is_empty() || (self.partition_kinds[p] != PartitionKind::Static && (!model.part_cells[p].is_empty() || !model.part_sag[p].is_empty())))
            .collect();
        for n in 0..program.node_count {
            let cells = model.note_cells.len() as u32;
            model.note_cells.extend_from_slice(&node_cells[n]);
            for &k in &node_supply[n] {
                let e = &model.sag_ics[k as usize];
                model.note_cells.extend(e.first as u32..(e.first + e.outputs.len()) as u32);
            }
            let leds = model.led_on_node.len() as u32;
            model.led_on_node.extend_from_slice(&led_nodes[n]);
            let part = self.local_index[n].map(|(p, _)| p).filter(|&p| model.part_noted[p]);
            model.node_note.push(NodeNote { cells, cells_end: model.note_cells.len() as u32, leds, leds_end: model.led_on_node.len() as u32, part: part.map_or(u32::MAX, |p| p as u32) });
        }
        model.load_laws = self.junctions.iter().map(|j| j.law).collect();
        model.any = !model.devs.is_empty() || !model.sag_ics.is_empty();
        self.slow_model = model;
        let mut trail_parts = vec![false; part_count];
        let mut trail_reads: Vec<Vec<usize>> = vec![Vec::new(); part_count];
        for sd in self.slow_model.devs.iter().filter(|sd| sd.class == Class::Stepped && !sd.cold && (self.slow.stress(sd.device).is_some() || self.slow.ptc(sd.device).is_some())) {
            let mut nodes = sd.nodes.clone();
            if let Some(e) = sd.led {
                nodes.extend(self.led_devices[e].junctions.iter().flat_map(|j| [j.anode, j.cathode]));
            }
            for &p in sd.parts.iter().filter(|&&p| self.partition_kinds[p] == PartitionKind::Stepped) {
                trail_parts[p] = true;
                trail_reads[p].extend(nodes.iter().copied().filter(|&n| self.local_index[n].is_none_or(|(q, _)| q != p)));
            }
        }
        for r in &mut trail_reads {
            r.sort_unstable();
            r.dedup();
        }
        self.stepped.trail_parts = trail_parts;
        self.stepped.trail_reads = trail_reads;
        let words = |n: usize| vec![0u64; n.div_ceil(64)];
        self.slow_scratch = SlowScratch {
            pub_flag: vec![false; part_count],
            ptc_since: vec![0.0; device_count],
            cell_bits: words(self.slow_model.cells.len()),
            cell_force: words(self.slow_model.cells.len()),
            cell_gate: vec![GATE_ANY; self.slow_model.cells.len()],
            dev_bits: words(self.slow_model.devs.len()),
            force_bits: words(self.slow_model.devs.len()),
            ..SlowScratch::default()
        };
    }

    // ---- the instant ---------------------------------------------------

    pub(super) fn slow_begin_instant(&mut self) {
        self.slow_scratch.journal = 1;
        self.slow_scratch.touched.clear();
        self.slow_scratch.ptc_log.clear();
    }

    pub(super) fn slow_end_instant(&mut self) {
        if self.slow_scratch.journal == 2 {
            self.slow.end_instant();
        }
        self.slow_scratch.journal = 0;
        self.slow_scratch.touched.clear();
        self.slow_scratch.ptc_log.clear();
    }

    /// The slow state for a write: the instant's journal is begun first if a
    /// window is open and it has not been (point 15's first-touch journal,
    /// begun at the first touch). Every write goes through here.
    fn sw(&mut self) -> &mut crate::slow::SlowState {
        if self.slow_scratch.journal == 1 {
            self.slow.begin_instant();
            self.slow_scratch.journal = 2;
        }
        &mut self.slow
    }

    /// The instant failed (point 15): the slow state goes back bit for bit,
    /// and every stamp it set this instant is set again from the state it
    /// went back to, which also clears those partitions' caches again (an
    /// entry the failed instant stored under its stamps must not be served).
    /// A window that bumped any `law_gen` clears the junction memo, since a
    /// generation number can recur for another law. The notes since the last
    /// settle are dropped with the instant that made them.
    pub(super) fn slow_rollback_instant(&mut self) {
        // The rolled-back instant may have failed, tripped or recovered a
        // device the probes were rebuilt for.
        if self.slow_scratch.journal == 2 {
            self.probe_version += 1;
        }
        let law_gen_bumped = self.slow_scratch.journal == 2 && self.slow.rollback_instant();
        self.slow_scratch.journal = 0;
        for &(dev, t) in self.slow_scratch.ptc_log.iter().rev() {
            self.slow_scratch.ptc_since[dev as usize] = t;
        }
        self.slow_scratch.ptc_log.clear();
        let touched = std::mem::take(&mut self.slow_scratch.touched);
        for &dev in &touched {
            if let Some(i) = self.slow_index(dev) {
                self.apply_stamp(i, false);
            }
        }
        if law_gen_bumped {
            self.static_solver.clear_junction_memo();
        }
        self.slow_scratch.rail_seg = None;
        // A gate may stand on a driver or a state the instant changed.
        self.slow_scratch.cell_gate.iter_mut().for_each(|g| *g = GATE_ANY);
        self.clear_slow_notes();
    }

    fn clear_slow_notes(&mut self) {
        let s = &mut self.slow_scratch;
        for &p in &s.pub_list {
            s.pub_flag[p as usize] = false;
        }
        s.pub_list.clear();
        if std::mem::take(&mut s.cells_any) {
            s.cell_bits.iter_mut().for_each(|w| *w = 0);
            s.cell_force.iter_mut().for_each(|w| *w = 0);
        }
        if std::mem::take(&mut s.devs_any) {
            s.dev_bits.iter_mut().for_each(|w| *w = 0);
            s.force_bits.iter_mut().for_each(|w| *w = 0);
        }
        s.driver_changes.clear();
        s.sag_due.clear();
        s.revisit.clear();
        s.dirty.clear();
        s.republish = false;
    }

    /// Notes the dynamic partitions `written[from..]` names as published
    /// (point 2): a closed-form segment start or re-aim, a stepped commit.
    /// The settle tells a stepped partition that committed at the instant
    /// from one that was only named.
    #[inline]
    pub(super) fn note_written_published(&mut self, from: usize) {
        for k in from..self.written.len() {
            let p = self.written[k] as usize;
            if self.slow_model.part_noted[p] && !self.slow_scratch.pub_flag[p] {
                self.slow_scratch.pub_flag[p] = true;
                self.slow_scratch.pub_list.push(p as u32);
            }
        }
    }

    /// A driver whose committed level the kernel just changed (a commit can
    /// flip a commanded direction without moving a node): its cell is noted
    /// at once, to be observed whatever its gate. A driver
    /// newer than the cell table waits in `driver_changes` for the settle to
    /// extend the table; a driver's cell never changes once it has one.
    #[inline]
    pub(super) fn note_driver_change(&mut self, idx: usize) {
        let s = &mut self.slow_scratch;
        match s.driver_cell.get(idx) {
            Some(&cell) => {
                if cell != u32::MAX {
                    s.cell_force[(cell >> 6) as usize] |= 1 << (cell & 63);
                    s.cells_any = true;
                }
            }
            None => s.driver_changes.push(idx as u32),
        }
    }

    fn slow_index(&self, dev: usize) -> Option<usize> {
        self.slow_model.of_device.get(dev).filter(|&&i| i != u32::MAX).map(|&i| i as usize)
    }

    // ---- the settle (point 2) ---------------------------------------------

    /// Point 2's settle and point 14's step 2: re-drives the slow devices of
    /// every partition published since the last settle (all of them at load),
    /// runs the `output_sag` scan and the due windows, and returns the
    /// partitions a latch due at once dirtied.
    ///
    /// An event that falls due at once inside a pass (a PTC recovering, a
    /// latch) asks for its device to be re-driven after its stamp change.
    /// When the pass dirtied a partition, the kernel's next delta cycle
    /// settles again at this instant; when it dirtied none (a device on
    /// fixed nodes, republished inside the pass), the re-drive runs here, in
    /// another pass. Each pass re-drives only devices whose event fired in
    /// the pass before, and a re-drive schedules a device's next event from
    /// the state its event just left, a positive time away for every kind;
    /// the passes are capped at one per slow device besides, and a request
    /// left past the cap waits for the next settle, as before.
    pub(super) fn settle_slow(&mut self, now: f64, drivers: &[Driver]) -> Result<Option<Vec<PartitionId>>, String> {
        self.note_rail_segment();
        if self.slow_model.any {
            self.refresh_stress_probes();
        }
        let scheduled = self.pending_schedule.len();
        let mut out = self.settle_pass(now, drivers)?;
        let mut passes = 0;
        while out.as_ref().is_none_or(|d| d.is_empty()) && !self.slow_scratch.revisit.is_empty() && passes < self.slow_model.devs.len() {
            passes += 1;
            out = self.settle_pass(now, drivers)?;
        }
        if out.as_ref().is_none_or(|d| d.is_empty()) {
            return Ok((self.pending_schedule.len() != scheduled).then(Vec::new));
        }
        Ok(out)
    }

    /// One pass of [`Self::settle_slow`].
    fn settle_pass(&mut self, now: f64, drivers: &[Driver]) -> Result<Option<Vec<PartitionId>>, String> {
        if !self.slow_model.any {
            self.clear_slow_notes();
            return Ok(None);
        }
        // Most instants of a digital board bring only node and driver notes
        // (the SAP-1 settles 14,000 times a simulated second): those visit
        // and observe what was noted and return, without the general path's
        // lists. Nothing noted at all returns at once.
        let sc = &self.slow_scratch;
        if sc.loaded
            && sc.pub_list.is_empty()
            && sc.revisit.is_empty()
            && sc.driver_changes.is_empty()
            && sc.sag_due.is_empty()
            && !sc.sag_all
            && !sc.republish
            && sc.dirty.is_empty()
            && self.slow_model.live_devs.is_empty()
            && self.slow_model.live_sag.is_empty()
        {
            if !sc.cells_any && !sc.devs_any {
                return Ok(None);
            }
            let scheduled = self.pending_schedule.len();
            self.close_stepped_trail(now);
            self.visit_noted(now);
            self.observe_noted(now, drivers);
            let sc = &self.slow_scratch;
            if sc.dirty.is_empty() && !sc.republish && self.pending_schedule.len() == scheduled {
                return Ok(None);
            }
            return self.finish_slow(now).map(Some);
        }
        let first = !self.slow_scratch.loaded;
        self.slow_scratch.loaded = true;
        let scheduled = self.pending_schedule.len();
        // The lists are taken for the call and handed back emptied, so their
        // buffers outlive it: a settle per instant allocates nothing.
        let mut published = std::mem::take(&mut self.slow_scratch.pub_list);
        let mut revisit = std::mem::take(&mut self.slow_scratch.revisit);
        {
            let s = &mut self.slow_scratch;
            let m = &self.slow_model;
            let mark = |s: &mut SlowScratch, i: u32, force: bool| {
                let (w, bit) = ((i >> 6) as usize, 1u64 << (i & 63));
                s.dev_bits[w] |= bit;
                if force {
                    s.force_bits[w] |= bit;
                }
                s.devs_any = true;
            };
            for &p in &published {
                s.pub_flag[p as usize] = false;
            }
            if first {
                for i in 0..m.devs.len() as u32 {
                    mark(s, i, true);
                }
            } else {
                for &p in &published {
                    let p = p as usize;
                    // A stepped partition named by a call but not committed
                    // at this instant took no step: nothing to close over.
                    if self.partition_kinds[p] == PartitionKind::Stepped && self.stepped.committed(p).is_none_or(|(t, _)| t != now) {
                        continue;
                    }
                    for &i in &m.part_devs[p] {
                        mark(s, i, false);
                    }
                }
                for &i in &m.live_devs {
                    mark(s, i, false);
                }
                for &i in &revisit {
                    mark(s, i, true);
                }
            }
        }
        self.close_stepped_trail(now);
        self.visit_noted(now);
        // A visit may have asked for another (a latch re-drives its device
        // after the re-solve it dirtied): those join the list handed back.
        revisit.clear();
        revisit.append(&mut self.slow_scratch.revisit);
        self.slow_scratch.revisit = revisit;

        // The `output_sag` scan, then the windows that ran out (point 9). A
        // cell is observed when one of its own inputs moved: its pin's node
        // or a supply's node published (noted as it did, node by node, for a
        // static partition; every cell a dynamic one holds or supplies), or
        // its driver changed (noted as it did); every cell of an IC whose
        // window ran out or that sits on a live node. Each at most once a
        // settle, in cell order: cells are independent of each other.
        let mut due = std::mem::take(&mut self.slow_scratch.sag_due);
        self.ensure_driver_cells(drivers);
        if first || std::mem::take(&mut self.slow_scratch.sag_all) {
            for cell in 0..self.slow_model.cells.len() as u32 {
                self.note_cell(cell);
            }
        } else {
            for n in 0..published.len() {
                let p = published[n] as usize;
                if self.partition_kinds[p] == PartitionKind::Static {
                    continue;
                }
                for m in 0..self.slow_model.part_cells[p].len() {
                    let cell = self.slow_model.part_cells[p][m];
                    self.note_cell(cell);
                }
                for m in 0..self.slow_model.part_sag[p].len() {
                    let k = self.slow_model.part_sag[p][m] as usize;
                    self.note_ic(k);
                }
            }
            for n in 0..self.slow_scratch.driver_changes.len() {
                let idx = self.slow_scratch.driver_changes[n] as usize;
                if let Some(&cell) = self.slow_scratch.driver_cell.get(idx).filter(|&&c| c != u32::MAX) {
                    self.note_cell(cell);
                }
            }
            for n in 0..self.slow_model.live_sag.len() {
                let k = self.slow_model.live_sag[n] as usize;
                self.note_ic(k);
            }
            for &k in &due {
                self.note_ic(k as usize);
            }
        }
        self.slow_scratch.driver_changes.clear();
        self.observe_noted(now, drivers);
        if !due.is_empty() {
            due.sort_unstable();
            due.dedup();
            for &k in &due {
                self.sag_windows(k as usize, now, drivers);
            }
            due.clear();
        }
        due.append(&mut self.slow_scratch.sag_due);
        self.slow_scratch.sag_due = due;
        published.clear();
        published.append(&mut self.slow_scratch.pub_list);
        self.slow_scratch.pub_list = published;
        let dirty = self.finish_slow(now)?;
        // Nothing scheduled and nothing dirtied: the kernel has nothing to
        // take from this settle, as when there was nothing to settle.
        if dirty.is_empty() && self.pending_schedule.len() == scheduled {
            return Ok(None);
        }
        Ok(Some(dirty))
    }

    /// Visits each noted device once, ascending, forced if any note forced it.
    #[inline]
    fn visit_noted(&mut self, now: f64) {
        // The trail's devices not otherwise noted still schedule their latch
        // from the drive it left.
        for k in 0..self.slow_scratch.trailed.len() {
            let i = self.slow_scratch.trailed[k];
            let (w, bit) = ((i >> 6) as usize, 1u64 << (i & 63));
            self.slow_scratch.dev_bits[w] |= bit;
            self.slow_scratch.devs_any = true;
        }
        if std::mem::take(&mut self.slow_scratch.devs_any) {
            for w in 0..self.slow_scratch.dev_bits.len() {
                let mut bits = std::mem::take(&mut self.slow_scratch.dev_bits[w]);
                let force = std::mem::take(&mut self.slow_scratch.force_bits[w]);
                while bits != 0 {
                    let b = bits.trailing_zeros();
                    bits &= bits - 1;
                    self.visit(w * 64 + b as usize, now, force >> b & 1 == 1);
                }
            }
        }
        self.slow_scratch.trailed.clear();
    }

    /// Observes each noted cell once, ascending, but for one noted by static
    /// publishes alone that its gate passes over ([`GATE_ANY`]).
    #[inline]
    fn observe_noted(&mut self, now: f64, drivers: &[Driver]) {
        if std::mem::take(&mut self.slow_scratch.cells_any) {
            self.ensure_driver_cells(drivers);
            for w in 0..self.slow_scratch.cell_bits.len() {
                let force = std::mem::take(&mut self.slow_scratch.cell_force[w]);
                let mut bits = std::mem::take(&mut self.slow_scratch.cell_bits[w]) | force;
                while bits != 0 {
                    let b = bits.trailing_zeros();
                    bits &= bits - 1;
                    let cell = w * 64 + b as usize;
                    if force >> b & 1 == 0 && self.gate_passes(cell) {
                        continue;
                    }
                    self.sag_observe(cell, now, drivers);
                }
            }
        }
    }

    /// Whether cell `cell`'s gate passes over a static publish of its nodes.
    #[inline]
    fn gate_passes(&self, cell: usize) -> bool {
        let dir = match self.slow_scratch.cell_gate[cell] {
            GATE_QUIET => return true,
            GATE_HIGH => sag::Dir::High,
            GATE_LOW => sag::Dir::Low,
            _ => return false,
        };
        let info = &self.slow_model.cells[cell];
        !sags(info, dir, self.voltages[info.node as usize])
    }

    /// The common tail of the settle, a slow event and a reset: the fixed
    /// devices' republish, then the dirtied partitions, ascending.
    fn finish_slow(&mut self, now: f64) -> Result<Vec<PartitionId>, String> {
        if std::mem::take(&mut self.slow_scratch.republish) {
            self.republish_fixed(now)?;
        }
        if self.slow_scratch.dirty.is_empty() {
            return Ok(Vec::new());
        }
        let mut dirty = std::mem::take(&mut self.slow_scratch.dirty);
        dirty.sort_unstable();
        dirty.dedup();
        Ok(dirty)
    }

    /// A device on fixed nodes changed its stamp (point 12): its current and
    /// the source KCL are recomputed at this instant, as the kernel's empty
    /// resolve recomputes them after a toggle of a fixed-node switch. The
    /// whole resolve, not the static solver's alone: a device straight on a
    /// rail changes the rail's load, and the rail's own loop must re-run at
    /// this instant (else the rail waits for the next publish, and the result
    /// depends on how the host chunks its advances).
    fn republish_fixed(&mut self, now: f64) -> Result<(), String> {
        let drivers = Rc::clone(&self.driver_snapshot);
        super::PartitionSolver::resolve_dirty(self, &[], now, &drivers).map(|_| ())
    }

    /// Ruling R71 (b) (ACC-A's 507/4 and 55/2): every step a stepped
    /// partition committed since the last settle closes its slow devices'
    /// stress and PTC integrators over that step, at the step's own end
    /// voltages and its own CC rail. The published values are not them: a
    /// re-seed at an event replaces them, a clamp or catch-up commits several
    /// steps before one publish, and a hub node is republished at the rail
    /// the loop moved to (55/2's PTC read 1 mV under its step's own end and
    /// tripped at the host's step). A latch the step's end drive reaches
    /// inside the step fires now, recorded where the integral crossed.
    /// Ruling R90 (a): every partition's rated stepped devices still able to
    /// fail (not failed, a PTC not tripped), handed to the stepped planner,
    /// which reads each trial's stress ratios from them
    /// (`SteppedSolver::set_stress_probes`). Rebuilt when one fails or a PTC
    /// trips or recovers (`probe_version`), so a plan never cuts for a device
    /// gone open or misses one come back.
    pub(super) fn refresh_stress_probes(&mut self) {
        if self.probes_at.is_some_and(|v| v == u64::MAX || v == self.probe_version) {
            return;
        }
        self.probes_at = Some(self.probe_version);
        let mut any = false;
        for p in 0..self.slow_model.part_devs.len() {
            let mut probes = Vec::new();
            for &i in &self.slow_model.part_devs[p] {
                let sd = &self.slow_model.devs[i as usize];
                if sd.class != Class::Stepped || sd.cold || self.slow.is_failed(sd.device) || sd.nodes.len() < 2 && sd.led.is_none() {
                    continue;
                }
                let (a, b) = (sd.nodes.first().copied().unwrap_or(0), sd.nodes.get(1).copied().unwrap_or(0));
                match sd.kind {
                    kinds::RESISTOR if sd.p_max > 0.0 => probes.push(stepped::StressProbe::Resistor { a, b, r: sd.record[0], r_power: sd.r_power, p_max: sd.p_max }),
                    kinds::FUSE => {
                        use kinds::fuse_param::*;
                        if sd.record[I_RATING] > 0.0 {
                            probes.push(stepped::StressProbe::Fuse { a, b, r_normal: sd.record[R_NORMAL], i_rating: sd.record[I_RATING] });
                        }
                    }
                    kinds::PTC_FUSE => {
                        if let Some(e) = self.slow.ptc(sd.device).filter(|e| !e.tripped) {
                            probes.push(stepped::StressProbe::Ptc { a, b, params: e.params });
                        }
                    }
                    kinds::RESISTOR => {}
                    _ => {
                        if let Some(e) = sd.led.map(|e| &self.led_devices[e]) {
                            if let Some(m) = led::if_max(&e.device).filter(|&m| m > 0.0) {
                                probes.push(stepped::StressProbe::Led { device: std::rc::Rc::new(e.device.clone()), junctions: std::rc::Rc::new(e.junctions.clone()), if_max: m });
                            }
                        }
                    }
                }
            }
            any |= !probes.is_empty();
            self.stepped.set_stress_probes(p, probes);
        }
        if !any && !self.slow_model.devs.iter().any(|sd| sd.class == Class::Stepped && !sd.cold && (self.slow.is_failed(sd.device) || self.slow.ptc(sd.device).is_some_and(|p| p.tripped))) {
            self.probes_at = Some(u64::MAX);
        }
    }

    /// A PTC trip or recovery fired at `now` through the settle's own path
    /// (`fire_since`), for a test that needs two in one settle.
    #[cfg(test)]
    pub(super) fn debug_ptc_event(&mut self, dev: usize, trip: bool, now: f64) {
        let i = self.slow_index(dev).expect("a slow device");
        self.fire_since(i, if trip { SlowEventKind::PtcTrip } else { SlowEventKind::PtcRecover }, now, now);
    }

    /// Ruling R90 (a)'s agreement check: for each rated stepped device, the
    /// ratio its drives carry and the one the stepped planner's probe reads,
    /// both at the published voltages.
    #[cfg(test)]
    pub(super) fn probe_agreement(&self, now: f64) -> Vec<(u32, f64, f64)> {
        let mut out = Vec::new();
        for p in 0..self.slow_model.part_devs.len() {
            let probes = self.stepped.debug_probe_list(p);
            let mut k = 0;
            for &i in &self.slow_model.part_devs[p] {
                let sd = &self.slow_model.devs[i as usize];
                if sd.class != Class::Stepped || sd.cold || self.slow.is_failed(sd.device) || k >= probes.len() {
                    continue;
                }
                let d = self.drives(i as usize, now);
                let q = match (d.stress, d.ptc) {
                    (Some(StressDrive::Constant { q, .. }), _) => q,
                    (_, Some(PtcDrive::Stress(StressDrive::Constant { q, .. }))) => q,
                    _ => continue,
                };
                if q < 0.0 {
                    continue;
                }
                let (pq, _) = probes[k].ratio(&|n| self.voltages[n]);
                out.push((sd.kind, q, pq));
                k += 1;
            }
        }
        out
    }

    /// Device `i`'s drives at committed step `c`'s start state (at `c.t0`)
    /// and at its end state (at `c.t1`), each read with the step's own
    /// voltages over the published ones.
    fn step_drives(&mut self, i: usize, c: &stepped::Committed) -> (Option<Drives>, Drives) {
        let start = c.start.as_ref().map(|s| self.drives_over(i, s, c.t0));
        let end: Vec<(usize, f64)> = c.nodes.iter().chain(c.rail.iter()).copied().collect();
        (start, self.drives_over(i, &end, c.t1))
    }

    fn drives_over(&mut self, i: usize, over: &[(usize, f64)], at: f64) -> Drives {
        let saved: Vec<(usize, f64)> = over.iter().map(|&(n, _)| (n, self.voltages[n])).collect();
        for &(n, v) in over {
            self.voltages[n] = v;
        }
        let drives = self.drives(i, at);
        for (n, v) in saved.into_iter().rev() {
            self.voltages[n] = v;
        }
        drives
    }

    fn close_stepped_trail(&mut self, now: f64) {
        if self.stepped.trail.is_empty() {
            return;
        }
        let trail = std::mem::take(&mut self.stepped.trail);
        for c in &trail {
            // Ruling R90 (a): the next step's reach from this step's devices,
            // and whether any of them is stressed.
            let mut reach = f64::INFINITY;
            let mut watch = false;
            for k in 0..self.slow_model.part_devs[c.p].len() {
                let i = self.slow_model.part_devs[c.p][k] as usize;
                let sd = &self.slow_model.devs[i];
                let dev = sd.device;
                if sd.class != Class::Stepped || sd.cold || self.slow.is_failed(dev) {
                    continue;
                }
                let (start, drives) = self.step_drives(i, c);
                watch |= drives.stress.is_some_and(|d| d.stressed()) || matches!(drives.ptc, Some(PtcDrive::Stress(d)) if d.stressed());
                if let Some(s0) = start.as_ref() {
                    let h = c.t1 - c.t0;
                    if let (Some(a), Some(b)) = (s0.stress, drives.stress) {
                        reach = reach.min(StressDrive::stress_reach(a, b, h));
                    }
                    if let (Some(PtcDrive::Stress(a)), Some(PtcDrive::Stress(b))) = (s0.ptc, drives.ptc) {
                        reach = reach.min(StressDrive::stress_reach(a, b, h));
                    }
                }
                let mut due = None;
                if let (Some(new), true) = (drives.stress, self.slow.stress(dev).is_some()) {
                    // Rulings R80 (a), R84 (B): the step holds its mean's
                    // pieces in order, and the end drive from its end.
                    let mean = start.as_ref().and_then(|d| d.stress).map_or(StepMean::whole(new), |s0| StressDrive::over_step(s0, new));
                    let s = self.sw().stress_mut(dev).expect("checked above");
                    match s.run_step(&mean, c.t0, c.t1) {
                        Some(t) => due = Some((t, SlowEventKind::StressLatch)),
                        None => s.drive = new,
                    }
                }
                if let Some(PtcDrive::Stress(new)) = drives.ptc.filter(|_| self.slow.ptc(dev).is_some_and(|p| !p.tripped)) {
                    let mean = match start.as_ref().and_then(|d| d.ptc) {
                        Some(PtcDrive::Stress(s0)) => StressDrive::over_step(s0, new),
                        _ => StepMean::whole(new),
                    };
                    let e = self.sw().ptc_mut(dev).expect("checked above");
                    match e.run_step(&mean, c.t0, c.t1) {
                        Some(t) if due.is_none_or(|(d, _)| t < d) => due = Some((t, SlowEventKind::PtcTrip)),
                        Some(_) => {}
                        None => e.set_drive(PtcDrive::Stress(new)),
                    }
                }
                self.slow_scratch.trailed.push(i as u32);
                if let Some((t, kind)) = due {
                    self.fire_since(i, kind, now, t);
                }
            }
            self.stepped.set_stress_reach(c.p, reach);
            self.stepped.set_stress_watch(c.p, watch);
        }
        let mut trail = trail;
        trail.clear();
        self.stepped.trail = trail;
    }

    /// When stepped device `i`'s integral crosses its latch on the drive its
    /// pending trial ends with (the first of its stepped partitions with
    /// one), from where it stands, and that trial's end; `None` without a
    /// pending trial. Writes nothing.
    fn pending_latch(&mut self, i: usize, now: f64) -> Option<(f64, f64)> {
        let _ = now;
        let dev = self.slow_model.devs[i].device;
        let c = self.slow_model.devs[i].parts.iter().filter(|&&p| self.partition_kinds[p] == PartitionKind::Stepped).find_map(|&p| self.stepped.pending_end(p, &self.voltages))?;
        // The trial's step is judged as its commit will close it (ruling R80
        // (a), R84 (B)): on the mean of its start and end drives.
        let (start, drives) = self.step_drives(i, &c);
        let stress = match (drives.stress, self.slow.stress(dev)) {
            (Some(drive), Some(s)) if !self.slow.is_failed(dev) => {
                let mean = start.as_ref().and_then(|d| d.stress).map_or(StepMean::whole(drive), |s0| StressDrive::over_step(s0, drive));
                let mut s = *s;
                s.run_step(&mean, c.t0, c.t1)
            }
            _ => None,
        };
        let trip = match (drives.ptc, self.slow.ptc(dev)) {
            (Some(PtcDrive::Stress(drive)), Some(e)) if !e.tripped => {
                let mean = match start.as_ref().and_then(|d| d.ptc) {
                    Some(PtcDrive::Stress(s0)) => StressDrive::over_step(s0, drive),
                    _ => StepMean::whole(drive),
                };
                let mut e = *e;
                e.run_step(&mean, c.t0, c.t1)
            }
            _ => None,
        };
        let t = match (stress, trip) {
            (Some(a), Some(b)) => a.min(b),
            (a, b) => a.or(b).unwrap_or(f64::INFINITY),
        };
        Some((t, c.t1))
    }

    /// The start of the step a stepped device's visit at `now` closes over
    /// (the latest of its stepped partitions committed at `now`), or `None`
    /// when none was: then the window runs from its last visit, as before.
    ///
    /// Defect B (cold 668/1): a visit that wrote nothing (an idle device, R38
    /// (a)) leaves the integrator's `t0` where it was, so the backward rule
    /// held what the next visit read over every quiet step since. A clock
    /// edge pulled a CC rail from 9.8 V to -1.4 V; rated R1-7 (84.7 ohm, 31
    /// mW), idle on an 8.6 uF settled at the old rail since 9.3 ms, read
    /// 47/s and was charged it back to then: S = 7 and burnt at the edge,
    /// where the discharge the edge starts gives S = 0.016. What a partition
    /// publishes after a re-solve at `now` is no step's end and starts at
    /// `now` (`SteppedSolver::step_start`).
    fn step_start(&self, i: usize, now: f64) -> Option<f64> {
        let sd = &self.slow_model.devs[i];
        sd.parts.iter().filter(|&&p| self.partition_kinds[p] == PartitionKind::Stepped).filter_map(|&p| self.stepped.step_start(p, now)).reduce(f64::max)
    }

    /// One device's visit (point 2): close each integrator over what drove
    /// it, set the new drive, handle what is due at once, and schedule the
    /// next event. A device whose drives are unchanged is left alone, so a
    /// quiet device is never written; `force` re-drives and reschedules it
    /// anyway (its pending event was consumed, or a reset made it stale).
    fn visit(&mut self, i: usize, now: f64, force: bool) {
        // R38 (a): a cold device's drive is idle at every voltage the circuit
        // can reach, so a visit, forced or not, would write nothing.
        if self.slow_model.devs[i].cold || (!force && led_quiet(&self.slow_model, &self.slow, &self.voltages, i)) {
            return;
        }
        let (dev, class) = (self.slow_model.devs[i].device, self.slow_model.devs[i].class);
        let backward = class == Class::Stepped;
        // The backward rule holds what this visit reads over the step just
        // committed, and over no more: up to the step's start each
        // integrator runs on the drive it had.
        let from = if backward { self.step_start(i, now) } else { None };
        // The trail closed it over the step it just committed, at the step's
        // own voltages, and left that drive: a re-seed at `now` starts the
        // published one instead.
        let kept = backward && from.is_some_and(|t| t < now) && self.slow_scratch.trailed.contains(&(i as u32));
        let failed = self.slow.is_failed(dev);
        let drives = self.drives(i, now);
        let mut wrote = false;
        if let (Some(new), Some(old)) = (drives.stress, self.slow.stress(dev).copied()) {
            // R38 (a): a closed-form segment that stays under the rate's zero
            // level is idle too, unless it is a rail-driven anchor, whose
            // exponential holds only until its re-anchor.
            let new_quiet = !new.accumulates() || (drives.redrive_by == f64::INFINITY && !new.accumulates_after(now));
            let idle = old.s0 == 0.0 && !old.drive.accumulates_after(old.t0) && new_quiet;
            if kept {
                wrote = true;
            } else if !failed && !idle && (backward || old.drive != new) {
                let s = self.sw().stress_mut(dev).expect("checked above");
                if backward {
                    if let Some(t) = from {
                        s.close(t);
                    }
                    s.drive = new;
                    s.close(now);
                } else {
                    s.close(now);
                    s.drive = new;
                }
                wrote = true;
            }
        }
        if let (Some(new), Some(old)) = (drives.power, self.slow.thermal(dev).copied()) {
            let zero = PowerDrive::Constant(0.0);
            let idle = old.temp0 == old.cfg.ambient_c && old.drive == zero && new == zero;
            if !idle && (backward || old.drive != new) {
                let th = self.sw().thermal_mut(dev).expect("checked above");
                if backward {
                    if let Some(t) = from {
                        th.close(t);
                    }
                    th.drive = new;
                    th.close(now);
                } else {
                    th.close(now);
                    th.drive = new;
                }
                wrote = true;
            }
            // Point 4, stepped partitions: `T_hold := T` after every committed
            // step, classic's one-step lag, with no event and no re-seed.
            let th = *self.slow.thermal(dev).expect("checked above");
            if backward && self.slow_model.devs[i].kind == kinds::LED && !failed && th.t_hold.to_bits() != th.temp0.to_bits() {
                self.sw().thermal_mut(dev).expect("checked above").t_hold = th.temp0;
                self.slow_counts[3] += 1;
                self.stamp_changed(i, now, false);
            }
        }
        if let (Some(new), Some(old)) = (drives.ptc, self.slow.ptc(dev).copied()) {
            let idle = !old.tripped && old.stress == 0.0 && matches!((old.drive, new), (PtcDrive::Stress(a), PtcDrive::Stress(b)) if !a.accumulates() && !b.accumulates());
            if kept && !old.tripped {
                wrote = true;
            } else if !idle && (backward || !ptc_same(old.drive, new, now)) {
                let e = self.sw().ptc_mut(dev).expect("checked above");
                if backward {
                    if let Some(t) = from {
                        e.close(t);
                    }
                    e.set_drive(new);
                    e.close(now);
                } else {
                    e.close(now);
                    e.set_drive(new);
                }
                wrote = true;
            }
        }
        // What is due at once happens inside this instant (point 5): a latch,
        // a boundary, a trip or a recovery. A device changes state at each, so
        // this ends; the bound only guards a broken invariant.
        for _ in 0..8 {
            match self.slow.next_event(dev) {
                Some((t, kind)) if is_due(t, now) => {
                    self.fire(i, kind, now);
                    wrote = true;
                }
                _ => break,
            }
        }
        if wrote {
            self.slow_counts[0] += 1;
        }
        // Ruling R71 (b): a stepped device's latch is scheduled from the drive
        // its step ended with, so it fires where its stress crosses inside the
        // step that follows; at the end of a step it crossed in, it latches at
        // that commit. It schedules nothing else between steps.
        if backward && (wrote || force) {
            let token = self.sw().bump_token(dev);
            if let Some((t, _)) = self.slow.next_latch(dev) {
                self.pending_schedule.push((t, EventKind::Slow { device: DeviceId(dev as u32), token: token & !crate::events::SLOW_ANCHOR }));
            }
        }
        if !backward && (wrote || force) {
            let next = self.sw().reschedule(dev);
            if let Some((t, _, token)) = next {
                // The token's top bit marks a re-anchor, so a device's own
                // event never carries it (a token wraps in 2^31 reschedules).
                self.pending_schedule.push((t, EventKind::Slow { device: DeviceId(dev as u32), token: token & !crate::events::SLOW_ANCHOR }));
            }
            // R21: the re-anchor, under the same token, so any later visit
            // that reschedules retires it. Only one before the device's own
            // event, which re-drives it anyway, and only for a drive this
            // visit wrote: a quiet device is not re-anchored.
            let by = drives.redrive_by.max(now + REDRIVE_MIN_DT);
            if wrote && drives.redrive_by.is_finite() && next.is_none_or(|(t, _, _)| by < t) {
                let token = self.slow.token(dev) | crate::events::SLOW_ANCHOR;
                self.pending_schedule.push((by, EventKind::Slow { device: DeviceId(dev as u32), token }));
            }
        }
    }

    /// A due event of device `i` at `now` (points 3-10): the event is the
    /// transition, so nothing is re-tested against its threshold.
    fn fire(&mut self, i: usize, kind: SlowEventKind, now: f64) {
        self.fire_since(i, kind, now, now);
    }

    /// [`Self::fire`], its latch or trip recorded at `since` (at or before
    /// `now`): where a stepped device's integral crossed inside a step
    /// committed earlier in the instant (ruling R71 (b)).
    fn fire_since(&mut self, i: usize, kind: SlowEventKind, now: f64, since: f64) {
        let dev = self.slow_model.devs[i].device;
        match kind {
            SlowEventKind::StressLatch => {
                if let Some(s) = self.sw().stress_mut(dev) {
                    s.close(now);
                }
                let (code, value, limit) = self.latch_values(i, now);
                self.sw().latch(LatchRecord { device: dev as u32, code: code as u32, pin: -1, since, value, limit, aux: 0.0 }, true);
                self.probe_version += 1;
                self.slow_counts[2] += 1;
                self.stamp_changed(i, now, true);
            }
            SlowEventKind::ThermalBoundary => {
                self.sw().thermal_mut(dev).expect("a boundary needs a thermal entry").rearm(now);
                self.slow_counts[3] += 1;
                self.stamp_changed(i, now, true);
            }
            SlowEventKind::ShutdownTrip | SlowEventKind::ShutdownRestart => {
                self.sw().thermal_mut(dev).expect("a shutdown needs a thermal entry").fire_shutdown(now);
                self.slow_scratch.revisit.push(i as u32);
            }
            SlowEventKind::PtcTrip => {
                let e = self.sw().ptc_mut(dev).expect("a trip needs a PTC");
                e.close(now);
                e.trip(now);
                self.probe_version += 1;
                let old = self.slow_scratch.ptc_since[dev];
                if self.window {
                    self.slow_scratch.ptc_log.push((dev as u32, old));
                }
                self.slow_scratch.ptc_since[dev] = since;
                self.slow_counts[2] += 1;
                self.stamp_changed(i, now, true);
            }
            SlowEventKind::PtcRecover => {
                let e = self.sw().ptc_mut(dev).expect("a recovery needs a PTC");
                e.close(now);
                e.recover(now);
                self.probe_version += 1;
                self.slow_counts[2] += 1;
                self.stamp_changed(i, now, true);
            }
        }
    }

    /// The one entry point for a slow change that alters a stamp (point 12,
    /// `mark_device_changed`): bumps the junction's `law_gen`, sets the
    /// device's stamp everywhere it is read ([`Self::apply_stamp`]), and with
    /// `dirty` marks every partition it touches dirty (their caches are
    /// cleared by the static solver), or, with none, republishes its current
    /// and the source KCL at this instant. A stepped `T_hold` lag passes
    /// `dirty = false`: it forces no re-seed (point 4).
    fn stamp_changed(&mut self, i: usize, now: f64, dirty: bool) {
        let _ = now;
        let dev = self.slow_model.devs[i].device;
        self.sw().bump_law_gen(dev);
        self.apply_stamp(i, true);
        if self.window && !self.slow_scratch.touched.contains(&dev) {
            self.slow_scratch.touched.push(dev);
        }
        if !dirty {
            return;
        }
        let parts = &self.slow_model.devs[i].parts;
        if parts.is_empty() {
            self.slow_scratch.republish = true;
        }
        self.slow_scratch.dirty.extend(parts.iter().map(|&p| PartitionId(p as u32)));
        // A device straight from the rail to a fixed node is a direct device
        // of the hub, which no partition re-solve reports (Rails point 2), as
        // a switch toggle there is.
        if let Some(hub) = self.hub.as_mut() {
            let direct = hub.direct_devices.binary_search(&dev).is_ok() || hub.direct_junctions.iter().any(|&k| self.junctions[k as usize].device == dev);
            if direct {
                hub.dirty = true;
            }
        }
        self.slow_scratch.revisit.push(i as u32);
    }

    /// Device `i`'s stamp, set from the slow state wherever it is read: a
    /// resistor's, fuse's or PTC's params with the slow flag
    /// (`devices::slow_flag`: the static base, `live_params` for a dynamic
    /// partition, the KCL contributions), an LED device's junctions (failed
    /// ones dropped, a held law applied) in the static solver, the stepped
    /// shapes, the source KCL list and the readout. A pure function of the
    /// slow state, so a rollback sets it again from the restored state, with
    /// `clear_caches` off: the static solver's window has put its caches back.
    fn apply_stamp(&mut self, i: usize, clear_caches: bool) {
        let sd = &self.slow_model.devs[i];
        let (dev, kind) = (sd.device, sd.kind);
        let failed = self.slow.is_failed(dev);
        let tripped = self.slow.ptc(dev).is_some_and(|p| p.tripped);
        let law_gen = self.slow.law_gen(dev);
        match kind {
            kinds::RESISTOR | kinds::FUSE | kinds::PTC_FUSE => {
                let flag = failed || tripped;
                let params = devices::with_slow_flag(kind, &sd.record, flag);
                let (dynamic_owner, kcl) = (sd.dynamic_owner, sd.kcl);
                if dynamic_owner {
                    if flag {
                        self.live_params.insert(dev, params.clone());
                    } else {
                        self.live_params.remove(&dev);
                    }
                } else {
                    self.set_static_stamp(dev, StampState { failed, tripped, law: None, law_gen }, clear_caches);
                }
                if let Some(pos) = kcl {
                    let kd = &mut self.kcl_devices[pos];
                    kd.contributions = KclDevice::resolve_contributions(kd.kind, &kd.nodes, &params);
                }
            }
            _ => {
                let law = self.held_law(i);
                if self.slow_model.devs[i].static_stamps {
                    self.set_static_stamp(dev, StampState { failed, tripped: false, law, law_gen }, clear_caches);
                }
                for k in 0..self.junctions.len() {
                    if self.junctions[k].device != dev {
                        continue;
                    }
                    let base = law.unwrap_or(self.slow_model.load_laws[k]);
                    // A failed junction is skipped by every KCL walk this solver
                    // runs; the zero law keeps the hub's direct-device sums from
                    // carrying its old current (W1f's `direct_at`).
                    self.junctions[k].law = if failed { led::JunctionLaw { is: 0.0, ..base } } else { base };
                }
                if let (Some(law), Some(e)) = (law, self.slow_model.devs[i].led) {
                    for j in &mut self.led_devices[e].junctions {
                        j.law = law;
                    }
                    if !super::junction_current_bounded(&law) {
                        self.lazy_static_ok = false;
                    }
                }
                let lists: Vec<(usize, Vec<DynJunction>)> = self
                    .slow_model
                    .stepped_junctions
                    .iter()
                    .filter(|(_, full)| full.iter().any(|(d, _)| *d == dev))
                    .map(|(p, full)| {
                        let list = full
                            .iter()
                            .filter(|(d, _)| !self.slow.is_failed(*d))
                            .map(|(d, j)| {
                                let mut j = *j;
                                if let Some(law) = self.slow_index(*d).and_then(|k| self.held_law(k)) {
                                    j.law = law;
                                }
                                j
                            })
                            .collect();
                        (*p, list)
                    })
                    .collect();
                for (p, list) in lists {
                    self.stepped.set_junctions(p, list);
                }
            }
        }
    }

    /// The static solver's half of a stamp change: `mark_device_changed`
    /// (point 12), or, putting a rolled-back state back, the same stamps with
    /// the caches its window already restored.
    fn set_static_stamp(&mut self, dev: usize, state: StampState, clear_caches: bool) {
        if clear_caches {
            self.static_solver.mark_device_changed(dev, state);
        } else {
            self.static_solver.set_device_stamp(dev, state, false);
        }
    }

    /// A plain `led` with a thermal identity: its law at `T_hold` (point 4).
    fn held_law(&self, i: usize) -> Option<led::JunctionLaw> {
        let sd = &self.slow_model.devs[i];
        if sd.kind != kinds::LED || !sd.thermal {
            return None;
        }
        let t_hold = self.slow.thermal(sd.device)?.t_hold;
        led::led_law_at(&self.led_devices[sd.led?].device, t_hold)
    }

    // ---- drives ------------------------------------------------------------

    /// A node's voltage as a closed form for a closed-form device: `None`
    /// when its segment is not one exponential (the caller holds the
    /// published value instead).
    ///
    /// On a segment a known rail drives (ruling R21) it is the segment's own
    /// exponential re-anchored at `now` through the node's exact value, with
    /// the instant by which it must be anchored again
    /// ([`super::closed_form::ClosedFormSolver::driven_anchor`]), each pin's
    /// share of the tolerance being half of [`DRIVE_TOL_V`] plus
    /// [`DRIVE_TOL_REL`]; infinity for any other node.
    fn node_v(&self, node: usize, now: f64) -> Option<(NodeV, f64)> {
        if self.local_index[node].is_some_and(|(p, _)| self.partition_kinds[p] == PartitionKind::ClosedForm) {
            if let Some((v_inf, v0, t0, tau)) = self.closed_form.node_trajectory(node) {
                return Some((NodeV::Exp(ExpVoltage::from_trajectory(v_inf, v0, t0, tau)), f64::INFINITY));
            }
            let (v_inf, v_now, tau, by) = self.closed_form.driven_anchor(node, now, 0.5 * DRIVE_TOL_V, 0.5 * DRIVE_TOL_REL)?;
            Some((NodeV::Exp(ExpVoltage::from_trajectory(v_inf, v_now, now, tau)), now + by))
        } else {
            Some((NodeV::Fixed(self.voltages[node]), f64::INFINITY))
        }
    }

    /// `V(pins[0]) - V(pins[1])` over a closed-form segment (points 3 and 5),
    /// anchored at `now`, with the instant it must be anchored again by.
    fn segment_volts(&self, i: usize, now: f64) -> Option<(ExpVoltage, f64)> {
        let sd = &self.slow_model.devs[i];
        if matches!(sd.class, Class::Static | Class::FixedPair) {
            return self.rail_volts(i, now);
        }
        if sd.class != Class::ClosedForm {
            return None;
        }
        // R29: on a linear segment a rail drives, the device's own voltage,
        // anchored with the lag in its asymptote (`driven_device`).
        let rail = self.hub.as_ref().map(|h| h.node);
        let driven = sd.parts.iter().copied().find(|&p| self.partition_kinds[p] == PartitionKind::ClosedForm && self.closed_form.is_driven(p));
        if let Some(p) = driven {
            let pin = |node: usize| {
                if self.local_index[node].is_some_and(|(q, _)| q == p) {
                    DevicePin::Node(node)
                } else if Some(node) == rail {
                    DevicePin::Rail
                } else {
                    DevicePin::Fixed(self.voltages[node])
                }
            };
            let pins = [(pin(sd.nodes[0]), 1.0), (pin(sd.nodes[1]), -1.0)];
            if let Some((v_inf, v_at, tau, by)) = self.closed_form.driven_device(p, &pins, now, DEVICE_TOL_V, DEVICE_TOL_REL) {
                return Some((ExpVoltage::from_trajectory(v_inf, v_at, now, tau), now + by));
            }
        }
        let ((mut a, mut by_a), (mut b, mut by_b)) = (self.node_v(sd.nodes[0], now)?, self.node_v(sd.nodes[1], now)?);
        // A pin on the rail of a partition the rail drives (R21): the rail's
        // exact value now, anchored like the other pin, not the last one the
        // hub published.
        if let Some(rail) = rail {
            if let Some(p) = driven.filter(|_| sd.nodes[0] == rail || sd.nodes[1] == rail) {
                if let Some((u, by)) = self.closed_form.driven_rail(p, now, 0.5 * DRIVE_TOL_V, 0.5 * DRIVE_TOL_REL) {
                    if sd.nodes[0] == rail {
                        (a, by_a) = (NodeV::Fixed(u), now + by);
                    } else {
                        (b, by_b) = (NodeV::Fixed(u), now + by);
                    }
                }
            }
        }
        let v = match (a, b) {
            (NodeV::Exp(a), NodeV::Exp(b)) => a.minus(b),
            (NodeV::Exp(a), NodeV::Fixed(vb)) => a.offset(vb),
            (NodeV::Fixed(va), NodeV::Exp(b)) => b.from_fixed(va),
            (NodeV::Fixed(_), NodeV::Fixed(_)) => return None,
        };
        Some((v, by_a.min(by_b)))
    }

    /// R30: notes every static or fixed-pair device the rail reaches when the
    /// CC rail capacitor's segment is not the one the last settle saw.
    fn note_rail_segment(&mut self) {
        let Some(hub) = self.hub.as_ref() else { return };
        if self.slow_scratch.rail_seg.as_ref().is_some_and(|s| *s == hub.state.cap_seg) {
            return;
        }
        self.slow_scratch.rail_seg = Some(hub.state.cap_seg.clone());
        let rail = hub.node;
        for i in 0..self.slow_model.devs.len() {
            let sd = &self.slow_model.devs[i];
            if !sd.cold && matches!(sd.class, Class::Static | Class::FixedPair) && sd.nodes.iter().any(|&n| n == rail || self.static_solver.hub_node_line(n).is_some()) {
                self.slow_scratch.dev_bits[i >> 6] |= 1 << (i & 63);
                self.slow_scratch.devs_any = true;
            }
        }
    }

    /// Ruling R30: a static or fixed-pair device's voltage while the CC rail
    /// capacitor's segment moves the rail, `V = c + σ u` over its pins (the
    /// rail, a static hub node's line `a + s u`, a fixed value). On a segment
    /// that is one exponential (no coupled modes) and lines that are exact (a
    /// linear hub partition), `V` is that exponential: exact, with nothing to
    /// re-anchor. Otherwise it is held at its value now, `c + σ u(now)`, for
    /// `tol / (|σ| S)`, `S = Σ|s_k|` bounding the rail's slope from now on.
    /// `None` without a segment (the rail moves only at its events, which
    /// publish) or for a device the rail does not reach (`σ = 0`).
    fn rail_volts(&self, i: usize, now: f64) -> Option<(ExpVoltage, f64)> {
        let hub = self.hub.as_ref()?;
        let seg = hub.state.cap_seg.as_ref()?;
        let rail = hub.node;
        let sd = &self.slow_model.devs[i];
        let (mut c, mut sigma, mut exact) = (0.0, 0.0, true);
        for (&node, sign) in sd.nodes.iter().zip([1.0, -1.0]) {
            if node == rail {
                sigma += sign;
            } else if let Some((a, s)) = self.static_solver.hub_node_line(node) {
                exact &= self.static_solver.hub_node_line_exact(node).is_some();
                c += sign * a;
                sigma += sign * s;
            } else {
                c += sign * self.voltages[node];
            }
        }
        if sigma == 0.0 {
            return None;
        }
        if exact && seg.modes.is_none() && seg.k > 0.0 {
            // `u = u0 + slope φ_k(t - t0)`, as a ramp from `c + σ u0`: its
            // asymptote `u0 + slope/k` can lie far past where CV ends the
            // segment, and `a + b e^(-s/tau)` would cancel (W1gV3).
            let v = ExpVoltage::ramp(c + sigma * seg.u0, -sigma * seg.slope / seg.k, 1.0 / seg.k, seg.t0);
            return Some((v, f64::INFINITY));
        }
        let v_at = c + sigma * seg.at(now);
        let slope: f64 = seg.modes_at(now).iter().map(|&(_, s)| s.abs()).sum();
        let tol = DEVICE_TOL_V + DEVICE_TOL_REL * v_at.abs();
        let by = if slope > 0.0 { now + tol / (sigma.abs() * slope) } else { f64::INFINITY };
        Some((ExpVoltage::new(v_at, 0.0, 1.0, now), by))
    }

    /// A node's voltage at `t` inside the instant: a closed-form partition's
    /// trajectory, or the published value.
    pub(super) fn slow_voltage(&self, node: usize, t: f64) -> f64 {
        match self.node_v(node, t) {
            Some((NodeV::Exp(v), _)) => v.at(t),
            _ => self.voltages[node],
        }
    }

    /// The two-terminal voltage of device `i` at `t`.
    fn device_volts(&self, i: usize, t: f64) -> f64 {
        let sd = &self.slow_model.devs[i];
        match self.segment_volts(i, t) {
            Some((v, _)) => v.at(t),
            None => self.voltages[sd.nodes[0]] - self.voltages[sd.nodes[1]],
        }
    }

    /// Device `i`'s drives from what was just published (points 3 and 5-10),
    /// each in classic's operation order.
    fn drives(&self, i: usize, now: f64) -> Drives {
        let sd = &self.slow_model.devs[i];
        let dev = sd.device;
        let failed = self.slow.is_failed(dev);
        let (seg, redrive_by) = match self.segment_volts(i, now) {
            Some((v, by)) => (Some(v), by),
            None => (None, f64::INFINITY),
        };
        let mut out = Drives { redrive_by, ..Drives::default() };
        match sd.kind {
            kinds::RESISTOR => {
                let r = sd.record[0];
                let dv = self.device_volts(i, now);
                if sd.p_max > 0.0 {
                    out.stress = Some(match seg {
                        // rate = k V^2 - 1 with k = rPower/(R^2 pMax) (point 5).
                        Some(v) => StressDrive::Segment(SegmentRate { k: if sd.r_power > 0.0 { sd.r_power / (r * r * sd.p_max) } else { 0.0 }, c: 1.0, recovery: failures::RESISTOR_RECOVERY, v }),
                        None => {
                            let d = failures::resistor_overload(dv, r, sd.r_power, sd.p_max);
                            StressDrive::Constant { rate: d.rate, recovery: d.recovery, zero: 1.0, q: failures::resistor_q(dv, r, sd.r_power, sd.p_max) }
                        }
                    });
                }
                if sd.thermal {
                    // `V^2/R` with the stamped R, and 0 once open (point 3).
                    out.power = Some(match (failed, seg) {
                        (true, _) => PowerDrive::Constant(0.0),
                        (false, Some(v)) => PowerDrive::Squared { gain: 1.0 / r, v },
                        (false, None) => PowerDrive::Constant(dv * dv / r),
                    });
                }
            }
            kinds::FUSE => {
                use kinds::fuse_param::*;
                let (r_normal, i_rating) = (sd.record[R_NORMAL], sd.record[I_RATING]);
                out.stress = Some(match seg {
                    Some(v) => {
                        let k = if i_rating > 0.0 { 1.0 / ((r_normal * i_rating) * (r_normal * i_rating)) } else { 0.0 };
                        StressDrive::Segment(SegmentRate { k, c: 1.0, recovery: failures::FUSE_RECOVERY, v })
                    }
                    None => {
                        let dv = self.device_volts(i, now);
                        let d = failures::fuse_tripped(dv, r_normal, i_rating);
                        StressDrive::Constant { rate: d.rate, recovery: d.recovery, zero: 1.0, q: failures::fuse_q(dv, r_normal, i_rating) }
                    }
                });
            }
            kinds::PTC_FUSE => {
                let Some(entry) = self.slow.ptc(dev) else { return out };
                let params = entry.params;
                out.ptc = Some(match seg {
                    Some(v) if entry.tripped => params.tripped_segment(v, now),
                    Some(v) => PtcDrive::Stress(params.untripped_segment(v)),
                    None => {
                        let current = self.device_volts(i, now).abs() / params.resistance(entry.tripped);
                        if entry.tripped {
                            // The backward rule's cool window starts where the
                            // step does, so the step's cool time counts.
                            let from = if sd.class == Class::Stepped { self.step_start(i, now).map_or(entry.t0, |t| t.max(entry.t0)) } else { now };
                            params.tripped_constant(current, from)
                        } else {
                            PtcDrive::Stress(params.untripped_constant(current))
                        }
                    }
                });
            }
            _ => {
                let Some(e) = sd.led.map(|e| &self.led_devices[e]) else { return out };
                if !failed {
                    let below = led_below_safe(&self.slow_model, &self.voltages, i);
                    out.stress = Some(if below {
                        // Every junction at or under the voltage where it
                        // carries `ifMax`: `peakI <= ifMax`, so the rate is
                        // `max(0, r^2 - 1)` with `r <= 1`, exactly 0. A
                        // stepped LED's step mean needs `r^2` itself where its
                        // current crosses `ifMax` inside a step (read at the
                        // level, a step from 3 ifMax^2 to 0.5 gained 0.82 for
                        // 0.12), so it alone pays for the peak current here.
                        let q = match led::if_max(&e.device) {
                            Some(m) if m > 0.0 && sd.class == Class::Stepped => failures::led_q(led::peak_current(&e.device, &e.junctions, &self.voltages), m),
                            _ => -1.0,
                        };
                        StressDrive::Constant { rate: 0.0, recovery: failures::LED_RECOVERY, zero: 1.0, q }
                    } else {
                        let peak = led::peak_current(&e.device, &e.junctions, &self.voltages);
                        let if_max = led::if_max(&e.device).unwrap_or(0.0);
                        let d = failures::led_failed(peak, if_max);
                        StressDrive::Constant { rate: d.rate, recovery: d.recovery, zero: 1.0, q: failures::led_q(peak, if_max) }
                    });
                }
                if sd.thermal {
                    // `max(0, V I(V))` at the held law, 0 once failed or while
                    // burnt (point 3); a burnt `led` has no junction.
                    let p = match e.junctions.first() {
                        Some(j) if !failed && !e.absent() => {
                            let v = self.voltages[j.anode] - self.voltages[j.cathode];
                            (v * j.law.current(v)).max(0.0)
                        }
                        _ => 0.0,
                    };
                    out.power = Some(PowerDrive::Constant(p));
                }
            }
        }
        out
    }

    /// A latch's record values at `now`: the drive's value (the power, the
    /// current, `peakI`) and its limit (points 6-8).
    fn latch_values(&self, i: usize, now: f64) -> (FailureKind, f64, f64) {
        let sd = &self.slow_model.devs[i];
        match sd.kind {
            kinds::RESISTOR => {
                let d = failures::resistor_overload(self.device_volts(i, now), sd.record[0], sd.r_power, sd.p_max);
                (FailureKind::ResistorOverload, d.value, d.limit)
            }
            kinds::FUSE => {
                use kinds::fuse_param::*;
                let d = failures::fuse_tripped(self.device_volts(i, now), sd.record[R_NORMAL], sd.record[I_RATING]);
                (FailureKind::FuseTripped, d.value, d.limit)
            }
            _ => {
                let e = &self.led_devices[sd.led.expect("an LED-family slow device")];
                let peak = led::peak_current(&e.device, &e.junctions, &self.voltages);
                (FailureKind::LedFailed, peak, led::if_max(&e.device).unwrap_or(0.0))
            }
        }
    }

    // ---- output_sag (point 9) ------------------------------------------

    /// IC `k`'s power at `now` (W1c point 1): its constant one when both
    /// supplies are constant.
    fn sag_power(&self, k: usize, now: f64) -> IcPower {
        let e = &self.slow_model.sag_ics[k];
        let (device, ic) = &self.ics[e.pos];
        // A hold overrides even a constant supply's power (W1c point 1a): the
        // hold procedure never holds such an IC, but a hold is a verdict, not
        // a supply reading.
        let hold = self.power_hold(*device);
        if let Some(p) = e.const_power {
            return if hold.is_free() { p } else { IcPower { powered: digital::power::verdict(ic, f64::NAN, f64::NAN, hold), ..p } };
        }
        digital::power::ic_power_held(ic, self.slow_voltage(ic.vcc.node, now), self.slow_voltage(ic.gnd.node, now), hold)
    }

    /// Cell `cell`'s commanded direction, if a target, and its pin's voltage
    /// at `now`.
    fn sag_read(&self, cell: usize, power: &IcPower, now: f64, drivers: &[Driver]) -> (Option<sag::Dir>, f64) {
        let info = self.slow_model.cells[cell];
        let target = drivers.get(info.driver as usize).and_then(|d| sag::driver_target(d, power.powered));
        let v = if info.cf { self.slow_voltage(info.node as usize, now) } else { self.voltages[info.node as usize] };
        (target, v)
    }

    /// Which cell each driver drives, and which driver each cell is: cell `c`
    /// of an IC is its `c`-th driver (`ics::register` adds them in output
    /// order).
    #[inline]
    fn ensure_driver_cells(&mut self, drivers: &[Driver]) {
        if self.slow_scratch.driver_cell.len() != drivers.len() && !self.slow_model.cells.is_empty() {
            self.derive_driver_cells(drivers);
        }
    }

    #[inline(never)]
    fn derive_driver_cells(&mut self, drivers: &[Driver]) {
        let driver_count = drivers.len();
        if self.ic_driver_ids_len != driver_count {
            self.ensure_ic_driver_ids(drivers);
        }
        let mut by_driver = vec![u32::MAX; driver_count];
        let m = &mut self.slow_model;
        m.cells.iter_mut().for_each(|c| c.driver = u32::MAX);
        for e in &m.sag_ics {
            for (c, &id) in self.ic_driver_ids[e.pos].iter().enumerate().take(e.outputs.len()) {
                if let Some(slot) = by_driver.get_mut(id as usize) {
                    *slot = (e.first + c) as u32;
                    m.cells[e.first + c].driver = id;
                }
            }
        }
        self.slow_scratch.driver_cell = by_driver;
    }

    /// Notes a cell for the next settle's scan, once.
    fn note_cell(&mut self, cell: u32) {
        let s = &mut self.slow_scratch;
        s.cell_force[(cell >> 6) as usize] |= 1 << (cell & 63);
        s.cells_any = true;
    }

    /// Notes every cell of IC `k`.
    fn note_ic(&mut self, k: usize) {
        let e = &self.slow_model.sag_ics[k];
        let (first, n) = (e.first as u32, e.outputs.len() as u32);
        for cell in first..first + n {
            self.note_cell(cell);
        }
    }

    /// One scan observation of cell `cell` (point 9, "At each scan"), once a
    /// settle: its state moves only when the observation changes it, and a
    /// fresh `Pending` schedules its window's end.
    fn sag_observe(&mut self, cell: usize, now: f64, drivers: &[Driver]) {
        let info = self.slow_model.cells[cell];
        let driver = drivers.get(info.driver as usize);
        let k = info.k as usize;
        // Most observations leave an `Ok` cell `Ok` (30,000 a second on the
        // SAP-1): a driver that commands no level is no target whatever the
        // voltage or the power (`sag::target`), and a commanded pin that does
        // not sag observes nothing. Both are decided here as `target` and
        // `observed` decide them, reading a constant power in place.
        // Each also leaves the cell's gate for the next static publish. A
        // gate other than `GATE_ANY` stands only on an `Ok` cell.
        if self.slow_scratch.cell_gate[cell] != GATE_ANY || matches!(self.slow.sag_cell(cell).state, SagState::Ok) {
            debug_assert!(matches!(self.slow.sag_cell(cell).state, SagState::Ok));
            let dir = match driver {
                Some(d) if d.stage != crate::drivers::DriverStage::Ne555Dis => match d.committed() {
                    Some(Mode::Low) => sag::Dir::Low,
                    Some(Mode::High) if !d.open_collector => sag::Dir::High,
                    _ => {
                        self.slow_scratch.cell_gate[cell] = GATE_QUIET;
                        return;
                    }
                },
                _ => {
                    self.slow_scratch.cell_gate[cell] = GATE_QUIET;
                    return;
                }
            };
            // A held IC's constant power is not its verdict: the slow path
            // below judges it (`sag_power`).
            if !info.cf && info.power != POWER_LIVE && self.holds_active == 0 {
                let v = self.voltages[info.node as usize];
                debug_assert_eq!(info.power == POWER_ON && sags(&info, dir, v), self.slow_model.sag_ics[k].const_power.is_some_and(|p| sag::read(dir, v, &p).is_some()));
                if info.power == POWER_OFF {
                    self.slow_scratch.cell_gate[cell] = GATE_QUIET;
                    return;
                }
                if !sags(&info, dir, v) {
                    self.slow_scratch.cell_gate[cell] = match dir {
                        sag::Dir::High => GATE_HIGH,
                        sag::Dir::Low => GATE_LOW,
                    };
                    return;
                }
            }
        }
        let power = self.sag_power(k, now);
        let target = driver.and_then(|d| sag::driver_target(d, power.powered));
        let v = if info.cf { self.slow_voltage(info.node as usize, now) } else { self.voltages[info.node as usize] };
        let observed = sag::observed(target, v, &power);
        let before = self.slow.sag_cell(cell).state;
        let mut after = before;
        let tr = after.observe(now, observed);
        // Quiet only for a driver that commands no level at any power: a
        // supply that moves can power a target up.
        let commands = driver.and_then(|d| sag::driver_target(d, true)).is_some();
        self.slow_scratch.cell_gate[cell] = if after == SagState::Ok && !commands { GATE_QUIET } else { GATE_ANY };
        if after == before {
            return;
        }
        let device = self.slow_model.sag_ics[k].device;
        *self.sw().sag_state_mut(cell) = after;
        if tr.publish == Publish::Unlatched {
            let pin = self.slow.sag_cell(cell).pin as i32;
            self.sw().clear_latch(LatchKey { device: device as u32, code: FailureKind::OutputSag as u32, pin });
        }
        if let Some(t) = tr.schedule {
            // An IC's token never moves: every window it schedules stays
            // live, and the event checks which of its cells are due.
            let token = self.slow.token(device);
            self.pending_schedule.push((t, EventKind::Slow { device: DeviceId(device as u32), token: token & !crate::events::SLOW_ANCHOR }));
        }
    }

    /// The windows of IC `k` that ran out at `now` (point 9): decided here,
    /// after the instant's delta cycle and the scan, against the state then.
    /// A pin still a target commanded in the pending direction, still
    /// sagging, latches; a flip at this instant has already restarted it.
    fn sag_windows(&mut self, k: usize, now: f64, drivers: &[Driver]) {
        let power = self.sag_power(k, now);
        let (device, first, n) = {
            let e = &self.slow_model.sag_ics[k];
            (e.device, e.first, e.outputs.len())
        };
        for cell in first..first + n {
            let before = self.slow.sag_cell(cell).state;
            let SagState::Pending { dir, .. } = before else { continue };
            let (target, v) = self.sag_read(cell, &power, now, drivers);
            let reading = if target == Some(dir) { sag::read(dir, v, &power) } else { None };
            let mut after = before;
            let tr = after.on_window(now, reading.is_some());
            if after == before {
                continue;
            }
            *self.sw().sag_state_mut(cell) = after;
            if tr.publish == Publish::Latched {
                let reading = reading.expect("a latch holds");
                let pin = self.slow.sag_cell(cell).pin;
                let rec = reading.record(device, pin, now);
                self.sw().latch(LatchRecord { device: device as u32, code: FailureKind::OutputSag as u32, pin: pin as i32, since: now, value: rec.value, limit: rec.limit, aux: rec.aux }, false);
                self.slow_counts[2] += 1;
            }
        }
    }

    // ---- events, reset, records, state -------------------------------------

    /// A current re-anchor (ruling R21): the device is re-driven at the
    /// settle, from its rail-driven segments' exact values now, and fires
    /// nothing.
    pub(super) fn slow_anchor(&mut self, dev: usize, now: f64) -> Result<Vec<PartitionId>, String> {
        self.slow_counts[1] += 1;
        if let Some(i) = self.slow_index(dev) {
            self.slow_scratch.revisit.push(i as u32);
        }
        self.finish_slow(now)
    }

    /// A due, current `Slow` event (point 14).
    pub(super) fn slow_event(&mut self, dev: usize, now: f64) -> Result<Vec<PartitionId>, String> {
        self.slow_counts[1] += 1;
        if let Some(&k) = self.slow_model.sag_of_device.get(dev).filter(|&&k| k != u32::MAX) {
            self.slow_scratch.sag_due.push(k);
            return Ok(Vec::new());
        }
        let Some(i) = self.slow_index(dev) else { return Ok(Vec::new()) };
        let stepped = self.slow_model.devs[i].class == Class::Stepped;
        let next = if stepped { self.slow.next_latch(dev) } else { self.slow.next_event(dev) };
        if let (true, Some(_)) = (stepped, next) {
            // The latch was predicted from the drive the last committed step
            // ended with, held forward. The step now running decides: its
            // trial's own end drive, over it, crosses before `now` or it does
            // not cross here (ruling R71 (b): the crossing inside its step).
            // Held forward, a falling rate latched what it never reaches
            // (cold 535 tpl 2: an LED charging 2.3 mF at the CC limit, 1603/s
            // held past the CC exit, latched at 15.0 ms where by hand its
            // damage tops out at 22.8 of 24).
            match self.pending_latch(i, now) {
                Some((t, _)) if is_due(t, now) => {}
                Some((t, end)) if t <= end => {
                    let token = self.sw().bump_token(dev);
                    self.pending_schedule.push((t, EventKind::Slow { device: DeviceId(dev as u32), token: token & !crate::events::SLOW_ANCHOR }));
                    return self.finish_slow(now);
                }
                Some(_) => {
                    let _ = self.sw().bump_token(dev);
                    return self.finish_slow(now);
                }
                None => {}
            }
        }
        if let Some((_, kind)) = next {
            if stepped {
                // A latch inside a step: the device's stepped partitions come
                // to it with the stamps they had, as a waveform edge brings
                // the partitions it feeds (W1b rule 6), on the CC rail their
                // trials were planned on; the re-solve the latch dirties then
                // starts them there.
                let parts: Vec<usize> = self.slow_model.devs[i].parts.iter().copied().filter(|&p| self.partition_kinds[p] == PartitionKind::Stepped).collect();
                let drivers = Rc::clone(&self.driver_snapshot);
                self.stepped.catch_up_on_cc_rail(&parts, now, &drivers, &self.live_params, &self.voltages)?;
                self.stepped.catch_up(&parts, now, &drivers, &self.live_params, &self.voltages)?;
            }
            self.fire(i, kind, now);
        }
        // Re-driven and rescheduled at the settle, once the partitions the
        // event dirtied have re-solved.
        self.slow_scratch.revisit.push(i as u32);
        self.finish_slow(now)
    }

    /// `resetFailures` (point 13): every latch, stress integrator, PTC and
    /// `output_sag` window cleared, `T` and `T_hold` kept, the latch log kept;
    /// each device whose stamp returns goes through the one entry point, and
    /// every IC is scanned afresh from this instant.
    pub(super) fn reset_slow(&mut self, now: f64) -> Result<Vec<PartitionId>, String> {
        let effects = self.sw().reset_failures(now);
        self.probe_version += 1;
        for c in 0..self.slow.sag_len() {
            if self.slow.sag_cell(c).state != SagState::Ok {
                *self.sw().sag_state_mut(c) = SagState::Ok;
            }
        }
        for dev in effects.restamped {
            if let Some(i) = self.slow_index(dev) {
                self.stamp_changed(i, now, true);
            }
        }
        for dev in effects.rescheduled {
            if let Some(i) = self.slow_index(dev) {
                self.slow_scratch.revisit.push(i as u32);
            }
        }
        self.slow_scratch.sag_all = true;
        self.slow_scratch.cell_gate.iter_mut().for_each(|g| *g = GATE_ANY);
        self.finish_slow(now)
    }

    /// `bo_engine_records` (C ABI): channel 0 the failures in latch order,
    /// an `output_sag` projected to `now` (point 9, "Record"); 1 the Rust
    /// warnings (none in W1); 2 the tripped PTCs; 3 the latch log, taken.
    pub(super) fn slow_records(&mut self, channel: u32, now: f64, out: &mut Vec<f64>) -> bool {
        out.clear();
        let push = |out: &mut Vec<f64>, r: &LatchRecord| out.extend([r.device as f64, r.code as f64, r.pin as f64, r.since, r.value, r.limit, r.aux]);
        match channel {
            0 => {
                let latched: Vec<LatchRecord> = self.slow.latched().to_vec();
                for r in &latched {
                    if r.code != FailureKind::OutputSag as u32 {
                        push(out, r);
                        continue;
                    }
                    if let Some(rec) = self.sag_projection(r, now) {
                        push(out, &rec);
                    }
                }
                true
            }
            1 => {
                warnings::collect(out);
                true
            }
            2 => {
                for sd in &self.slow_model.devs {
                    if self.slow.ptc(sd.device).is_some_and(|p| p.tripped) {
                        out.extend([sd.device as f64, 1.0, -1.0, self.slow_scratch.ptc_since[sd.device], 0.0, 0.0, 0.0]);
                    }
                }
                true
            }
            3 => {
                for r in self.slow.take_latch_log() {
                    push(out, &r);
                }
                true
            }
            _ => false,
        }
    }

    /// A latched `output_sag` as read at `now`: `since = now` and the value
    /// and limit at `now`, or `None` when its predicate no longer holds (a
    /// closed-form segment crossed back since the last publish).
    fn sag_projection(&self, r: &LatchRecord, now: f64) -> Option<LatchRecord> {
        let k = *self.slow_model.sag_of_device.get(r.device as usize).filter(|&&k| k != u32::MAX)? as usize;
        let e = &self.slow_model.sag_ics[k];
        let c = e.outputs.iter().position(|&li| li + 2 == r.pin as usize)?;
        let SagState::Latched { dir } = self.slow.sag_cell(e.first + c).state else { return None };
        let power = self.sag_power(k, now);
        let node = self.ics[e.pos].1.logic[e.outputs[c]].node;
        let reading = sag::read(dir, self.slow_voltage(node, now), &power)?;
        Some(LatchRecord { since: now, value: reading.actual, limit: reading.limit, aux: dir.aux(), ..*r })
    }

    /// `bo_engine_device_state`'s W1g rows (C ABI), projected to `now`
    /// without committing anything (point 16): a resistor's or `led`'s
    /// thermal state `[T, P, shutdown]` (none without a code 91 record), and
    /// a PTC's `[tripped, tripStress, recoveryTime]`.
    pub(super) fn slow_state_of(&self, device: usize, now: f64, out: &mut Vec<f64>) -> Option<usize> {
        out.clear();
        match *self.device_kinds.get(device)? {
            kinds::RESISTOR | kinds::LED => {
                if let Some(th) = self.slow.thermal(device) {
                    out.extend([th.temperature_at(now), th.power_at(now), f64::from(u8::from(th.shutdown))]);
                }
                Some(out.len())
            }
            kinds::PTC_FUSE => {
                let (tripped, stress, rt) = self.slow.ptc(device)?.projected(now);
                out.extend([f64::from(u8::from(tripped)), stress, rt]);
                Some(out.len())
            }
            _ => None,
        }
    }
}

