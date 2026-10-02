//! The digital core (core/README.md, "Kernel semantics: the digital core
//! (W1c)"). Orchestrator-owned: this file holds the typed view of an IC record
//! every submodule shares; the submodules belong to their W1c/W1d workers.
//!
//! - `power`, `levels`, `floating`: IC power, pin reads and the floating model
//!   (W1c worker C1). Pure functions of an [`IcRecord`] and voltages.
//! - `combinational`, `delay`: the 11 truth tables and the mapping of an
//!   evaluation onto driver requests (W1c worker C2).
//! - `sequential`, `memory`: the W1d register/memory families' own state and
//!   per-kind evaluators (D1/D2) — [`FamilyOutput`] is the interface
//!   `devices::ics`'s dispatcher (W1d worker K1) turns into `DriverRequest`s
//!   and digital-slot writes, mirroring `delay::driver_requests`'s job for
//!   the combinational family.
//! - `ne555`: the NE555's comparators, latch, output stages, element current
//!   and crossing thresholds (W1e worker E1). Pure functions of an
//!   [`IcRecord`], its latch and voltages.

pub mod combinational;
pub mod delay;
pub mod floating;
pub mod levels;
pub mod memory;
pub mod ne555;
pub mod power;
pub mod sag;
pub mod sequential;

use crate::drivers::DelaySet;
use crate::events::Mode;
use crate::kinds::{self, ic_param};
use crate::program::{Device, PIN_FLAG_OPEN, PIN_FLAG_OPEN_HIGH};

/// One logic pin of an IC record, in `icPinMap` order (core/README.md, "IC
/// logic pins").
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IcPin {
    pub node: usize,
    /// Classic's open-pin rule (pin record flag bit 0).
    pub open: bool,
    /// In `IC_OPEN_HIGH_PINS` (pin record flag bit 1).
    pub open_high: bool,
    /// FNV-1a state after `${id}:${pin}:` and after `${id}:${pin}:undefined:`.
    pub open_hash: u32,
    pub dead_hash: u32,
    /// Blob flag `FLOATING_INPUT`: an open pin of this kind gets a pull.
    pub floating_input: bool,
    /// Blob flag `OPEN_COLLECTOR`: a committed HIGH releases the pin.
    pub open_collector: bool,
}

/// A combinational IC record, validated by `program.rs` and typed here.
#[derive(Debug, Clone, PartialEq)]
pub struct IcRecord {
    pub kind: u32,
    pub vcc: IcPin,
    pub gnd: IcPin,
    pub logic: Vec<IcPin>,
    pub vcc_min: f64,
    pub vcc_max: f64,
    pub ttl: bool,
    pub cmos_ratio: bool,
    pub vil: Option<f64>,
    pub vih: Option<f64>,
    /// `(vtPlus, vtMinus)` for a Schmitt part.
    pub schmitt: Option<(f64, f64)>,
    pub nominal: f64,
    /// `None` means classic's plain 50 ohm output.
    pub io_max: Option<f64>,
    pub delay: f64,
    /// The first of this IC's digital slots.
    pub slot_base: usize,
    /// The delay tail's `(dH, dL)` pairs, in the kind's pair order
    /// (core/README.md, "IC delay tail"); empty for a kind without a tail.
    pub delays: Vec<(f64, f64)>,
    /// Params after the tail (the EEPROMs' `tWC`, `sdp`); empty otherwise.
    pub extra: Vec<f64>,
}

/// Whether `kind` is a W1d sequential IC (codes 70-72).
pub fn is_sequential_ic(kind: u32) -> bool {
    matches!(kind, kinds::IC_74LS161 | kinds::IC_74LS173 | kinds::IC_74HC74)
}

/// Whether `kind` is a W1d memory (codes 80-82).
pub fn is_memory_ic(kind: u32) -> bool {
    matches!(kind, kinds::IC_74LS189 | kinds::IC_28C16 | kinds::IC_28C256)
}

/// Whether `kind` is the NE555 (code 40), an IC for W1c points 1-3, 8 and 9
/// with its own evaluator (core/README.md, "Kernel semantics: NE555").
pub fn is_ne555(kind: u32) -> bool {
    kind == kinds::NE555
}

/// Whether `kind` is any IC with an `IcRecord`: combinational, sequential,
/// memory or the NE555. W1c's call sites that meant "an IC" use this.
pub fn is_ic(kind: u32) -> bool {
    is_combinational_ic(kind) || is_sequential_ic(kind) || is_memory_ic(kind) || is_ne555(kind)
}

/// Whether `kind` is one of the W1c combinational IC codes.
pub fn is_combinational_ic(kind: u32) -> bool {
    matches!(
        kind,
        kinds::IC_74LS00
            | kinds::IC_74LS04
            | kinds::IC_74LS08
            | kinds::IC_74LS32
            | kinds::IC_74LS86
            | kinds::IC_74LS157
            | kinds::IC_74LS283
            | kinds::IC_74HC138
            | kinds::IC_74HC14
            | kinds::IC_74LS47
            | kinds::IC_74LS245
    )
}

/// Whether `kind`'s output-capable pins are also READ, not driven-only (W1c
/// contract point 3's 74ls245 rule, generalised by W1d memories point 1 to
/// the EEPROMs' `io0..io7`): such a pin can never be crossing-watched (its
/// own IC's commit would race the watch — `solve::mod`'s
/// `register_ic_read_pin_watches` already excludes every output-capable pin
/// uniformly) and forces any dynamic partition holding one to be stepped
/// instead of closed-form (`partition::build`). The 74ls189's `/o` pins are
/// driven only (its address/data lines are separate read pins), so it is
/// NOT included here.
pub fn is_read_and_driven(kind: u32) -> bool {
    matches!(kind, kinds::IC_74LS245 | kinds::IC_28C16 | kinds::IC_28C256)
}

// ---- output_capable: every IC kind's driver footprint ---------------------
//
// Moved here from `combinational.rs` (W1d): this is the one table describing
// which logic-pin indices own a digital slot and a `Driver`, across every IC
// kind this crate knows, combinational and W1d sequential/memory alike.
// `combinational::output_capable` now just forwards to this copy, so its many
// existing call sites (and its own tests) keep compiling unchanged.

const OUT_QUAD3: [usize; 4] = [2, 5, 8, 11];
const OUT_HEX2: [usize; 6] = [1, 3, 5, 7, 9, 11];
const OUT_157: [usize; 4] = [4, 7, 10, 13];
const OUT_283: [usize; 5] = [9, 10, 11, 12, 13];
const OUT_138: [usize; 8] = [6, 7, 8, 9, 10, 11, 12, 13];
const OUT_47: [usize; 7] = [7, 8, 9, 10, 11, 12, 13];
const OUT_245: [usize; 16] = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17];
/// 74ls161: `/clr clk /load enp ent a b c d` (0-8, `r`), `qa qb qc qd rco`
/// (9-13, `o`) — core/README.md's sequential pin table.
const OUT_161: [usize; 5] = [9, 10, 11, 12, 13];
/// 74ls173: `/clr clk m n g1 g2 d1 d2 d3 d4` (0-9, `r`), `q1 q2 q3 q4` (10-13,
/// `o`).
const OUT_173: [usize; 4] = [10, 11, 12, 13];
/// 74hc74: `clr1_n d1 clk1 pre1_n` (0-3, `r`), `q1 q1_n q2_n q2` (4-7, `o`),
/// `pre2_n clk2 d2 clr2_n` (8-11, `r`).
const OUT_74HC74: [usize; 4] = [4, 5, 6, 7];
/// 74ls189: `a0 a1 a2 a3 d1 d2 d3 d4 /cs /we` (0-9, `r`), `/o1 /o2 /o3 /o4`
/// (10-13, `o`).
const OUT_189: [usize; 4] = [10, 11, 12, 13];
/// 28c16: `a0..a10 /oe /ce /we` (0-13, `r`), `io0..io7` (14-21, `o`/`r`).
const OUT_28C16: [usize; 8] = [14, 15, 16, 17, 18, 19, 20, 21];
/// 28c256: `a0..a14 /oe /ce /we` (0-17, `r`), `io0..io7` (18-25, `o`/`r`).
const OUT_28C256: [usize; 8] = [18, 19, 20, 21, 22, 23, 24, 25];

// NE555 logic indices, in the record's pin order after `8` and `1`
// (core/README.md, "NE555 record"). `digital::ne555` re-exports them under
// its locked names (`TRIG`, `OUT`, ...); they live here so this table and
// the evaluator share one definition.
pub const NE555_TRIG: usize = 0;
pub const NE555_OUT: usize = 1;
pub const NE555_RST: usize = 2;
pub const NE555_CV: usize = 3;
pub const NE555_THR: usize = 4;
pub const NE555_DIS: usize = 5;
/// OUT and DIS: each is its own driver stage, not a logic output.
const OUT_555: [usize; 2] = [NE555_OUT, NE555_DIS];

/// Indices into an IC's logic pins (pin order) that own a digital slot and a
/// `Driver`, across every IC kind — combinational (README's "IC logic pins"
/// table) and W1d sequential/memory (README's "Sequential and memory IC
/// logic pins" table) alike. `&[]` for a kind this crate does not know.
pub fn output_capable(kind: u32) -> &'static [usize] {
    match kind {
        kinds::IC_74LS00 | kinds::IC_74LS08 | kinds::IC_74LS32 | kinds::IC_74LS86 => &OUT_QUAD3,
        kinds::IC_74LS04 | kinds::IC_74HC14 => &OUT_HEX2,
        kinds::IC_74LS157 => &OUT_157,
        kinds::IC_74LS283 => &OUT_283,
        kinds::IC_74HC138 => &OUT_138,
        kinds::IC_74LS47 => &OUT_47,
        kinds::IC_74LS245 => &OUT_245,
        kinds::IC_74LS161 => &OUT_161,
        kinds::IC_74LS173 => &OUT_173,
        kinds::IC_74HC74 => &OUT_74HC74,
        kinds::IC_74LS189 => &OUT_189,
        kinds::IC_28C16 => &OUT_28C16,
        kinds::IC_28C256 => &OUT_28C256,
        kinds::NE555 => &OUT_555,
        _ => &[],
    }
}

/// One family evaluator's per-delta output (W1d contract, "Kernel semantics:
/// sequential ICs"/"...memories") — what `devices::ics`'s dispatcher turns
/// into `DriverRequest`s and digital-slot writes, the W1d counterpart of
/// `delay::driver_requests` for the combinational family.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FamilyOutput {
    /// One `(level, delays)` per [`output_capable`]`(kind)` pin, in that same
    /// order — always a concrete [`Mode`] (`Hiz` included: W1d delays point
    /// 4, "no IC requests a reset any more" past the combinational family).
    pub requests: Vec<(Mode, DelaySet)>,
    /// One entry per this kind's digital slot (`kinds::shape(kind).digital_slots`),
    /// `NaN` meaning absent — a DIFFERENT indexing from `requests`: the
    /// README's per-kind readout tables map some slots to state that has no
    /// driver of its own (a 161's bare `<id>` count) and leave some drivers
    /// with no slot at all (its `rco` pin).
    pub slots: Vec<f64>,
    /// `(at, tag)` pairs for `DriverRequest::Timer` — the EEPROMs' pending
    /// write-completion deadline (W1d, "Kernel semantics: memories", point
    /// 4.2 and point 5) is the one producer.
    pub timers: Vec<(f64, u64)>,
}

impl IcRecord {
    /// Types a device the reader already validated against its IC shape
    /// (pin, param and blob counts, flag codes, slot range), so nothing here
    /// can fail.
    pub fn from_device(device: &Device) -> IcRecord {
        let pin = |i: usize, blob_flags: u32| {
            let p = &device.pins[i];
            IcPin {
                node: p.node,
                open: p.flags & PIN_FLAG_OPEN != 0,
                open_high: p.flags & PIN_FLAG_OPEN_HIGH != 0,
                open_hash: p.open_hash,
                dead_hash: p.dead_hash,
                floating_input: blob_flags & kinds::PIN_FLOATING_INPUT != 0,
                open_collector: blob_flags & kinds::PIN_OPEN_COLLECTOR != 0,
            }
        };
        let p = &device.params;
        let fam = p[ic_param::FAM_FLAGS] as u32;
        IcRecord {
            kind: device.kind,
            vcc: pin(0, 0),
            gnd: pin(1, 0),
            logic: (2..device.pins.len()).map(|i| pin(i, device.blob[i - 1] as u32)).collect(),
            vcc_min: p[ic_param::VCC_MIN],
            vcc_max: p[ic_param::VCC_MAX],
            ttl: fam & kinds::FAM_TTL != 0,
            cmos_ratio: fam & kinds::FAM_CMOS_RATIO != 0,
            vil: (p[ic_param::VIL_FIXED] == 1.0).then_some(p[ic_param::VIL]),
            vih: (p[ic_param::VIH_FIXED] == 1.0).then_some(p[ic_param::VIH]),
            schmitt: (p[ic_param::SCHMITT] == 1.0).then_some((p[ic_param::VT_PLUS], p[ic_param::VT_MINUS])),
            nominal: p[ic_param::NOMINAL],
            io_max: (p[ic_param::IO_MAX] > 0.0).then_some(p[ic_param::IO_MAX]),
            delay: p[ic_param::DELAY],
            slot_base: device.blob[0] as usize,
            delays: (0..kinds::delay_pairs(device.kind)).map(|k| (p[ic_param::COUNT + 2 * k], p[ic_param::COUNT + 2 * k + 1])).collect(),
            extra: p[ic_param::COUNT + 2 * kinds::delay_pairs(device.kind)..].to_vec(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{Device, PinRecord};

    #[test]
    fn types_a_74ls47_record() {
        let pin = |node: usize, flags: u32| PinRecord { node, flags, open_hash: 7, dead_hash: 9 };
        let mut pins = vec![pin(1, 0), pin(0, 0)];
        pins.extend((0..14).map(|i| pin(2 + i, if i == 3 { PIN_FLAG_OPEN | PIN_FLAG_OPEN_HIGH } else { 0 })));
        let mut blob = vec![5.0];
        blob.extend((0..14).map(|i| if i >= 7 { kinds::PIN_OPEN_COLLECTOR as f64 } else { kinds::PIN_FLOATING_INPUT as f64 }));
        let device = Device {
            kind: kinds::IC_74LS47,
            component_index: 0,
            pins,
            params: vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.024, 35e-9],
            blob,
        };
        let ic = IcRecord::from_device(&device);
        assert_eq!(ic.logic.len(), 14);
        assert_eq!((ic.vcc.node, ic.gnd.node), (1, 0));
        assert!(ic.logic[3].open && ic.logic[3].open_high, "rbo_n is open and open-high");
        assert!(ic.logic[0].floating_input && !ic.logic[0].open_collector);
        assert!(ic.logic[7].open_collector, "e_out is open-collector");
        assert!(ic.ttl && !ic.cmos_ratio);
        assert_eq!((ic.vil, ic.vih), (Some(0.8), Some(2.0)));
        assert_eq!(ic.schmitt, None);
        assert_eq!(ic.io_max, Some(0.024));
        assert_eq!(ic.slot_base, 5);
        assert!(is_combinational_ic(ic.kind));
    }

    #[test]
    fn types_a_28c256_record_with_its_delay_pairs_and_extras() {
        let pin = |node: usize| PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 };
        let mut pins = vec![pin(1), pin(0)];
        pins.extend((0..26).map(|_| pin(2)));
        let mut params = vec![4.5, 5.5, 0.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.004, 0.0];
        params.extend([150e-9, 150e-9, 150e-9, 150e-9, 50e-9, 50e-9, 70e-9, 70e-9, 50e-9, 50e-9, 10e-3, 1.0]);
        let mut blob = vec![3.0];
        blob.extend(std::iter::repeat(1.0).take(26));
        blob.extend(std::iter::repeat(0.0).take(8192));
        let ic = IcRecord::from_device(&Device { kind: kinds::IC_28C256, component_index: 0, pins, params, blob });
        assert_eq!(ic.logic.len(), 26);
        assert_eq!(ic.delays.len(), 5);
        assert_eq!(ic.delays[3], (70e-9, 70e-9), "oeEnable is the fourth pair");
        assert_eq!(ic.extra, vec![10e-3, 1.0], "tWC then sdp");
        assert!(is_memory_ic(ic.kind) && is_ic(ic.kind) && !is_combinational_ic(ic.kind));
    }

    #[test]
    fn the_ne555_is_an_ic_driving_out_and_dis_only() {
        assert!(is_ne555(kinds::NE555) && is_ic(kinds::NE555));
        assert!(!is_combinational_ic(kinds::NE555) && !is_sequential_ic(kinds::NE555) && !is_memory_ic(kinds::NE555));
        assert_eq!(output_capable(kinds::NE555), &[1, 5]);
        assert_eq!(output_capable(kinds::NE555), &[ne555::OUT, ne555::DIS]);
        assert!(!is_read_and_driven(kinds::NE555), "DIS is driven, and THR is a separate read pin");
        assert_eq!((ne555::TRIG, ne555::RST, ne555::CV, ne555::THR), (0, 2, 3, 4));
        assert!(!is_ic(kinds::LED) && output_capable(kinds::SEG7_CA).is_empty());
    }
}
