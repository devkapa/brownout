//! The digital core (core/README.md, "Kernel semantics: the digital core
//! (W1c)"). Orchestrator-owned: this file holds the typed view of an IC record
//! every submodule shares; the submodules belong to their W1c workers.
//!
//! - `power`, `levels`, `floating`: IC power, pin reads and the floating model
//!   (W1c worker C1). Pure functions of an [`IcRecord`] and voltages.
//! - `combinational`, `delay`: the 11 truth tables and the mapping of an
//!   evaluation onto driver requests (W1c worker C2).

pub mod combinational;
pub mod delay;
pub mod floating;
pub mod levels;
pub mod power;

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
}
