//! Kind codes and per-kind record shapes (core/README.md, "Kind codes and
//! records"). The orchestrator owns this file: before a sub-wave that ports a
//! kind is dispatched, its code gets a shape here and a row in the README's
//! record-layout table, and workers never edit either. A code with no shape is
//! reserved, and the reader rejects it as `-2` (unknown kind).

pub const RESISTOR: u32 = 1;
pub const VOLTAGE_SOURCE: u32 = 2;
pub const CAPACITOR: u32 = 3;
pub const SWITCH: u32 = 4;
pub const PUSH_BUTTON: u32 = 5;
pub const SPDT_SWITCH: u32 = 6;
pub const DIP_SWITCH: u32 = 7;
pub const POTENTIOMETER: u32 = 8;
pub const TRIMMER: u32 = 9;
pub const FUSE: u32 = 10;
pub const LDR: u32 = 11;
pub const THERMISTOR: u32 = 12;

pub const BATTERY_PACK: u32 = 20;
pub const BENCH_PSU: u32 = 21;
pub const CLOCK: u32 = 22;
pub const CLOCK_GEN: u32 = 23;
pub const PULSE_SOURCE: u32 = 24;
pub const PULSE_GEN: u32 = 25;

pub const LED: u32 = 30;
pub const RGB_LED: u32 = 31;
pub const BICOLOR_LED: u32 = 32;
pub const SEG7_CC: u32 = 33;
pub const SEG7_CA: u32 = 34;

pub const NE555: u32 = 40;

pub const IC_74LS00: u32 = 50;
pub const IC_74LS04: u32 = 51;
pub const IC_74LS08: u32 = 52;
pub const IC_74LS32: u32 = 53;
pub const IC_74LS86: u32 = 54;
pub const IC_74LS157: u32 = 55;
pub const IC_74LS283: u32 = 56;
pub const IC_74HC138: u32 = 57;
pub const IC_74HC14: u32 = 58;
pub const IC_74LS47: u32 = 59;
pub const IC_CD4511: u32 = 60;
pub const IC_74LS245: u32 = 61;

pub const IC_74LS161: u32 = 70;
pub const IC_74LS173: u32 = 71;
pub const IC_74HC74: u32 = 72;
pub const IC_74HC595: u32 = 73;
pub const IC_74HC165: u32 = 74;
pub const IC_CD4017: u32 = 75;
pub const IC_CD4060: u32 = 76;

pub const IC_74LS189: u32 = 80;
pub const IC_28C16: u32 = 81;
pub const IC_28C256: u32 = 82;

/// How many blob values a kind's record carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlobShape {
    Exact(usize),
    /// Variable length with a floor, for contents such as ROM bytes.
    AtLeast(usize),
}

/// The record shape the program reader enforces for one kind.
#[derive(Debug, Clone, Copy)]
pub struct KindShape {
    pub code: u32,
    pub name: &'static str,
    /// Allowed pin-count range, inclusive. Most kinds have `pins == pins_max`;
    /// a kind whose pin count follows a param (dip_switch) has a range and
    /// pins it down exactly in `check`.
    pub pins: usize,
    pub pins_max: usize,
    pub params: usize,
    pub blob: BlobShape,
    /// Kind-specific checks on the pin count and the already-finite params
    /// and blob. `Err` names the problem; the reader reports it as `-1`.
    pub check: fn(usize, &[f64], &[f64]) -> Result<(), String>,
    /// Param indices `bo_engine_set_params` may change after load (owner
    /// decision 1). Anything else is `-8`.
    pub live_params: &'static [usize],
    /// How many digital readout slots a record with this pin count
    /// publishes. When non-zero, `blob[0]` is the first slot and the reader
    /// checks `blob[0] + count <= S` (core/README.md, "Slot indices").
    pub digital_slots: fn(usize) -> usize,
}

fn no_checks(_: usize, _: &[f64], _: &[f64]) -> Result<(), String> {
    Ok(())
}

fn no_slots(_: usize) -> usize {
    0
}

fn is_switch_value(v: f64) -> bool {
    v == 0.0 || v == 1.0
}

/// A non-positive R cannot come from the compiler (it clamps to 1e-3), only
/// from a hand-built program, and it would divide by zero in the stamp.
fn resistor_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if params[0] > 0.0 {
        Ok(())
    } else {
        Err(format!("resistor R must be > 0, got {}", params[0]))
    }
}

/// Params `[C, esr, leakage]`. Leakage `0` is the explicit code for "no
/// leakage" (classic's infinite parallel resistance), since params are
/// always finite.
fn capacitor_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !(params[0] > 0.0) {
        return Err(format!("capacitor C must be > 0, got {}", params[0]));
    }
    if params[1] < 0.0 || params[2] < 0.0 {
        return Err(format!("capacitor esr and leakage must be >= 0, got {} and {}", params[1], params[2]));
    }
    Ok(())
}

fn single_switch_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if is_switch_value(params[0]) {
        Ok(())
    } else {
        Err(format!("switch state must be 0 or 1, got {}", params[0]))
    }
}

/// Params `[k, sw1..sw8]`: `k` positions in `[2, 8]` and exactly `2k` pins
/// (`a1, b1, ..., ak, bk`). Every `sw` value is 0 or 1; those past `k` are
/// inert, as in classic.
fn dip_switch_check(pins: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    let k = params[0];
    if !(2.0..=8.0).contains(&k) || k.fract() != 0.0 {
        return Err(format!("dip_switch positions must be an integer in [2, 8], got {k}"));
    }
    if pins != 2 * (k as usize) {
        return Err(format!("dip_switch with {k} positions needs {} pins, got {pins}", 2 * (k as usize)));
    }
    if let Some(bad) = params[1..].iter().find(|v| !is_switch_value(**v)) {
        return Err(format!("dip_switch sw values must be 0 or 1, got {bad}"));
    }
    Ok(())
}

/// The params every W1c combinational IC carries, in order (core/README.md,
/// "IC params").
pub mod ic_param {
    pub const VCC_MIN: usize = 0;
    pub const VCC_MAX: usize = 1;
    pub const FAM_FLAGS: usize = 2;
    pub const VIL_FIXED: usize = 3;
    pub const VIL: usize = 4;
    pub const VIH_FIXED: usize = 5;
    pub const VIH: usize = 6;
    pub const SCHMITT: usize = 7;
    pub const VT_PLUS: usize = 8;
    pub const VT_MINUS: usize = 9;
    pub const NOMINAL: usize = 10;
    pub const IO_MAX: usize = 11;
    pub const DELAY: usize = 12;
    pub const COUNT: usize = 13;
}

/// `famFlags` bits.
pub const FAM_TTL: u32 = 1;
pub const FAM_CMOS_RATIO: u32 = 2;
/// Per-logic-pin blob flag bits.
pub const PIN_FLOATING_INPUT: u32 = 1;
pub const PIN_OPEN_COLLECTOR: u32 = 2;

fn is_code_in(v: f64, max: u32) -> bool {
    v.fract() == 0.0 && (0.0..=max as f64).contains(&v)
}

/// The shared IC checks: the flag params are codes, the physical ones are
/// in range, and every per-pin blob flag uses only the two defined bits.
/// The blob's slot base is checked by the reader against `S`.
fn ic_check(_: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    use ic_param::*;
    if !is_code_in(params[FAM_FLAGS], FAM_TTL | FAM_CMOS_RATIO) {
        return Err(format!("famFlags must be an integer in [0, 3], got {}", params[FAM_FLAGS]));
    }
    for (i, name) in [(VIL_FIXED, "vilFixed"), (VIH_FIXED, "vihFixed"), (SCHMITT, "schmitt")] {
        if !is_switch_value(params[i]) {
            return Err(format!("{name} must be 0 or 1, got {}", params[i]));
        }
    }
    for (i, name) in [(VIL, "vil"), (VIH, "vih"), (IO_MAX, "ioMax"), (DELAY, "delay")] {
        if params[i] < 0.0 {
            return Err(format!("{name} must be >= 0, got {}", params[i]));
        }
    }
    if let Some(bad) = blob.get(1..).unwrap_or(&[]).iter().find(|v| !is_code_in(**v, PIN_FLOATING_INPUT | PIN_OPEN_COLLECTOR)) {
        return Err(format!("pin flags must be an integer in [0, 3], got {bad}"));
    }
    Ok(())
}

/// Declares one combinational IC: `logic` pins after `vcc` and `gnd`, and
/// `outputs` of them publish a digital slot each.
macro_rules! ic_shape {
    ($code:expr, $name:expr, $logic:expr, $outputs:expr) => {{
        fn slots(_: usize) -> usize {
            $outputs
        }
        KindShape {
            code: $code,
            name: $name,
            pins: 2 + $logic,
            pins_max: 2 + $logic,
            params: ic_param::COUNT,
            blob: BlobShape::Exact(1 + $logic),
            check: ic_check,
            live_params: &[],
            digital_slots: slots,
        }
    }};
}

const SHAPES: &[KindShape] = &[
    KindShape { code: RESISTOR, name: "resistor", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: resistor_check, live_params: &[], digital_slots: no_slots },
    KindShape { code: VOLTAGE_SOURCE, name: "voltage_source", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: no_checks, live_params: &[], digital_slots: no_slots },
    KindShape { code: CAPACITOR, name: "capacitor", pins: 2, pins_max: 2, params: 3, blob: BlobShape::Exact(0), check: capacitor_check, live_params: &[], digital_slots: no_slots },
    KindShape { code: SWITCH, name: "switch", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots },
    KindShape { code: PUSH_BUTTON, name: "push_button", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots },
    KindShape { code: SPDT_SWITCH, name: "spdt_switch", pins: 3, pins_max: 3, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots },
    KindShape { code: DIP_SWITCH, name: "dip_switch", pins: 4, pins_max: 16, params: 9, blob: BlobShape::Exact(0), check: dip_switch_check, live_params: &[1, 2, 3, 4, 5, 6, 7, 8], digital_slots: no_slots },
    ic_shape!(IC_74LS00, "74ls00", 12, 4),
    ic_shape!(IC_74LS04, "74ls04", 12, 6),
    ic_shape!(IC_74LS08, "74ls08", 12, 4),
    ic_shape!(IC_74LS32, "74ls32", 12, 4),
    ic_shape!(IC_74LS86, "74ls86", 12, 4),
    ic_shape!(IC_74LS157, "74ls157", 14, 4),
    ic_shape!(IC_74LS283, "74ls283", 14, 5),
    ic_shape!(IC_74HC138, "74hc138", 14, 8),
    ic_shape!(IC_74HC14, "74hc14", 12, 6),
    ic_shape!(IC_74LS47, "74ls47", 14, 7),
    ic_shape!(IC_74LS245, "74ls245", 18, 16),
];

/// The shape for `code`, or `None` when the code is unknown or reserved.
pub fn shape(code: u32) -> Option<&'static KindShape> {
    SHAPES.iter().find(|s| s.code == code)
}
