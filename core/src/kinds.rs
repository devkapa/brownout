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
    pub pins: usize,
    pub params: usize,
    pub blob: BlobShape,
    /// Kind-specific checks on already-finite params. `Err` names the
    /// problem; the reader reports it as `-1`.
    pub check_params: fn(&[f64]) -> Result<(), String>,
}

fn no_param_checks(_: &[f64]) -> Result<(), String> {
    Ok(())
}

/// A non-positive R cannot come from the compiler (it clamps to 1e-3), only
/// from a hand-built program, and it would divide by zero in the stamp.
fn resistor_params(params: &[f64]) -> Result<(), String> {
    if params[0] > 0.0 {
        Ok(())
    } else {
        Err(format!("resistor R must be > 0, got {}", params[0]))
    }
}

const SHAPES: &[KindShape] = &[
    KindShape { code: RESISTOR, name: "resistor", pins: 2, params: 1, blob: BlobShape::Exact(0), check_params: resistor_params },
    KindShape { code: VOLTAGE_SOURCE, name: "voltage_source", pins: 2, params: 1, blob: BlobShape::Exact(0), check_params: no_param_checks },
];

/// The shape for `code`, or `None` when the code is unknown or reserved.
pub fn shape(code: u32) -> Option<&'static KindShape> {
    SHAPES.iter().find(|s| s.code == code)
}
