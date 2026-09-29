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
    /// How many channel readout slots a record with this pin count
    /// publishes. When non-zero, the blob carries the first channel slot
    /// right after the digital base (or at `blob[0]` with no digital slots),
    /// and the reader checks `base + count <= C`.
    pub channel_slots: fn(usize) -> usize,
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
/// The blob's slot base is checked by the reader against `S`. Pin flags sit
/// at `blob[1..=L]`, `L = pin_count - 2`; an EEPROM's ROM words follow them
/// and are checked as words by [`eeprom_check`].
fn ic_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
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
    let logic = pin_count.saturating_sub(2);
    if let Some(bad) = blob.get(1..=logic).unwrap_or(&[]).iter().find(|v| !is_code_in(**v, PIN_FLOATING_INPUT | PIN_OPEN_COLLECTOR)) {
        return Err(format!("pin flags must be an integer in [0, 3], got {bad}"));
    }
    Ok(())
}

/// A combinational IC with a datasheet delay tail (W1d retrofit: 74ls157,
/// 74ls245) on top of [`ic_check`]: every tail value (`params[13..]`) must be
/// `>= 0`, the same rule [`seq_check`] applies to the W1d kinds' tails. Every
/// param is already checked finite for every kind (`program.rs`, "every
/// param and blob value finite"), so only the range is this function's job.
fn ic_tail_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    ic_check(pin_count, params, blob)?;
    if let Some(bad) = params[ic_param::COUNT..].iter().find(|v| **v < 0.0) {
        return Err(format!("delay-tail values must be >= 0, got {bad}"));
    }
    Ok(())
}

/// Delay pairs in each IC kind's tail (core/README.md, "IC delay tail"):
/// params `13 + 2k` and `14 + 2k`. Kinds without a tail return 0.
pub fn delay_pairs(kind: u32) -> usize {
    match kind {
        IC_74LS157 => 1,
        IC_74LS245 => 2,
        IC_74LS161 | IC_74LS173 => 4,
        IC_74HC74 => 2,
        IC_74LS189 | IC_28C16 | IC_28C256 => 5,
        _ => 0,
    }
}

/// The EEPROMs' further params, after their delay tail.
pub mod eeprom_param {
    pub const TWC: usize = 23;
    pub const SDP: usize = 24;
    pub const COUNT: usize = 25;
}

/// ROM words per EEPROM image: 4 bytes per word (core/README.md, "ROM
/// image").
pub fn eeprom_words(kind: u32) -> usize {
    match kind {
        IC_28C16 => 2048 / 4,
        IC_28C256 => 32768 / 4,
        _ => 0,
    }
}

/// The sequential and memory kinds (codes 70-82) on top of [`ic_check`]:
/// classic reads them with no Schmitt branch and drives them push-pull, so
/// `schmitt` and `OPEN_COLLECTOR` must be clear; every delay-tail value is
/// `>= 0`.
fn seq_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    ic_check(pin_count, params, blob)?;
    if params[ic_param::SCHMITT] != 0.0 {
        return Err("schmitt must be 0 for a sequential or memory IC".to_string());
    }
    let logic = pin_count.saturating_sub(2);
    if blob.get(1..=logic).unwrap_or(&[]).iter().any(|v| (*v as u32) & PIN_OPEN_COLLECTOR != 0) {
        return Err("OPEN_COLLECTOR is not allowed on a sequential or memory IC".to_string());
    }
    // The tail is every param after 12, except an EEPROM's tWC and sdp,
    // which follow it and are checked by `eeprom_check`.
    let tail_end = if params.len() == eeprom_param::COUNT { eeprom_param::TWC } else { params.len() };
    if let Some(bad) = params[ic_param::COUNT..tail_end].iter().find(|v| **v < 0.0) {
        return Err(format!("delay-tail values must be >= 0, got {bad}"));
    }
    Ok(())
}

/// An EEPROM (codes 81, 82) on top of [`seq_check`]: `tWC > 0`, `sdp` is 0
/// or 1, and every ROM word after the pin flags is an integer in `[0, 2^32)`.
/// `sdp = 1` on a 28c16 is rejected by [`eeprom_28c16_check`].
fn eeprom_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    seq_check(pin_count, params, blob)?;
    if !(params[eeprom_param::TWC] > 0.0) {
        return Err(format!("tWC must be > 0, got {}", params[eeprom_param::TWC]));
    }
    if !is_switch_value(params[eeprom_param::SDP]) {
        return Err(format!("sdp must be 0 or 1, got {}", params[eeprom_param::SDP]));
    }
    let words_from = 1 + pin_count.saturating_sub(2);
    if let Some(bad) = blob.get(words_from..).unwrap_or(&[]).iter().find(|v| !(v.fract() == 0.0 && (0.0..4_294_967_296.0).contains(*v))) {
        return Err(format!("ROM words must be integers in [0, 2^32), got {bad}"));
    }
    Ok(())
}

fn eeprom_28c16_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    eeprom_check(pin_count, params, blob)?;
    if params[eeprom_param::SDP] != 0.0 {
        return Err("sdp must be 0 on a 28c16 (classic applies SDP to the 28c256 only)".to_string());
    }
    Ok(())
}

/// `led` params (core/README.md, "LED and display records").
pub mod led_param {
    pub const VF: usize = 0;
    pub const N: usize = 1;
    /// Also the Q-O24 damage limit: for `led` the two resolve identically.
    pub const I_RATED: usize = 2;
    pub const IS_SET: usize = 3;
    pub const IS: usize = 4;
    pub const BURNT: usize = 5;
    pub const COUNT: usize = 6;
}

/// `rgb_led` params.
pub mod rgb_param {
    pub const VF_R: usize = 0;
    pub const VF_G: usize = 1;
    pub const VF_B: usize = 2;
    pub const I_RATED: usize = 3;
    /// The Q-O24 damage limit, which reads specs where the stamp does not.
    pub const IF_MAX: usize = 4;
    pub const BURNT: usize = 5;
    pub const COUNT: usize = 6;
}

/// `bicolor_led` params.
pub mod bicolor_param {
    pub const VF1: usize = 0;
    pub const VF2: usize = 1;
    pub const I_RATED: usize = 2;
    /// The Q-O24 damage limit, as `rgb_param::IF_MAX`.
    pub const IF_MAX: usize = 3;
    pub const BURNT: usize = 4;
    pub const COUNT: usize = 5;
}

/// `seg7_cc` / `seg7_ca` params. No `burnt` and no damage limit: classic
/// gives `seg7` no failure surface (Q-O22).
pub mod seg7_param {
    pub const VF: usize = 0;
    pub const I_RATED: usize = 1;
    pub const COUNT: usize = 2;
}

fn burnt_check(burnt: f64) -> Result<(), String> {
    if is_switch_value(burnt) {
        Ok(())
    } else {
        Err(format!("burnt must be 0 or 1, got {burnt}"))
    }
}

/// `n <= 0` would divide by `n·Vt = 0` or invert the junction, which classic
/// fails to solve; the compiler declines it, so only a hand-built program
/// reaches here with one. `is` is 0 unless the `Is` override is set, so a
/// record has one encoding per value.
fn led_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    use led_param::*;
    if !(params[N] > 0.0) {
        return Err(format!("led n must be > 0, got {}", params[N]));
    }
    if !is_switch_value(params[IS_SET]) {
        return Err(format!("led isSet must be 0 or 1, got {}", params[IS_SET]));
    }
    if params[IS_SET] == 0.0 && params[IS] != 0.0 {
        return Err(format!("led is must be 0 when isSet is 0, got {}", params[IS]));
    }
    burnt_check(params[BURNT])
}

fn rgb_led_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    burnt_check(params[rgb_param::BURNT])
}

fn bicolor_led_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    burnt_check(params[bicolor_param::BURNT])
}

fn rgb_channels(_: usize) -> usize {
    3
}

fn bicolor_channels(_: usize) -> usize {
    2
}

fn seg7_channels(_: usize) -> usize {
    8
}

fn one_slot(_: usize) -> usize {
    1
}

/// Pins after `8` and `1` on an `ne555` (core/README.md, "NE555 record").
const NE555_LOGIC_PINS: usize = 6;

/// The NE555 on top of [`ic_check`]: params 7-9, 11 and 12 must be 0
/// (classic computes a Schmitt read, an `io_max` output resistance and a
/// package delay it never uses for this kind, so the record carries none),
/// and every per-pin flag must be 0 (no floating pull, and DIS is its own
/// driver stage rather than an open-collector logic output).
fn ne555_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    use ic_param::*;
    ic_check(pin_count, params, blob)?;
    for (i, name) in [(SCHMITT, "schmitt"), (VT_PLUS, "vtPlus"), (VT_MINUS, "vtMinus"), (IO_MAX, "ioMax"), (DELAY, "delay")] {
        if params[i] != 0.0 {
            return Err(format!("{name} must be 0 on an ne555, got {}", params[i]));
        }
    }
    if let Some(bad) = blob[1..=NE555_LOGIC_PINS].iter().find(|v| **v != 0.0) {
        return Err(format!("pin flags must be 0 on an ne555, got {bad}"));
    }
    Ok(())
}

/// Declares one combinational IC: `logic` pins after `vcc` and `gnd`, and
/// `outputs` of them publish a digital slot each. The 4-arg form is a plain
/// W1c kind (13 params, no tail); the 6-arg form is a kind with a W1d
/// datasheet delay tail of `pairs` pairs (13 + 2*pairs params) and its own
/// tail-checking `check` function (74ls157, 74ls245: [`ic_tail_check`]).
macro_rules! ic_shape {
    ($code:expr, $name:expr, $logic:expr, $outputs:expr) => {
        ic_shape!($code, $name, $logic, $outputs, 0, ic_check)
    };
    ($code:expr, $name:expr, $logic:expr, $outputs:expr, $pairs:expr, $check:expr) => {{
        fn slots(_: usize) -> usize {
            $outputs
        }
        KindShape {
            code: $code,
            name: $name,
            pins: 2 + $logic,
            pins_max: 2 + $logic,
            params: ic_param::COUNT + 2 * $pairs,
            blob: BlobShape::Exact(1 + $logic),
            check: $check,
            live_params: &[],
            digital_slots: slots,
            channel_slots: no_slots,
        }
    }};
}

/// Declares one sequential or memory IC (codes 70-82): `logic` pins after
/// `vcc` and `gnd`, `params` in all (13 + the tail + any extras), `blob`
/// values in all, and `slots` digital slots.
macro_rules! w1d_shape {
    ($code:expr, $name:expr, $logic:expr, $params:expr, $blob:expr, $slots:expr, $check:expr) => {{
        fn slots(_: usize) -> usize {
            $slots
        }
        KindShape {
            code: $code,
            name: $name,
            pins: 2 + $logic,
            pins_max: 2 + $logic,
            params: $params,
            blob: BlobShape::Exact($blob),
            check: $check,
            live_params: &[],
            digital_slots: slots,
            channel_slots: no_slots,
        }
    }};
}

const SHAPES: &[KindShape] = &[
    KindShape { code: RESISTOR, name: "resistor", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: resistor_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: VOLTAGE_SOURCE, name: "voltage_source", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: no_checks, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: CAPACITOR, name: "capacitor", pins: 2, pins_max: 2, params: 3, blob: BlobShape::Exact(0), check: capacitor_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: SWITCH, name: "switch", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: PUSH_BUTTON, name: "push_button", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: SPDT_SWITCH, name: "spdt_switch", pins: 3, pins_max: 3, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: DIP_SWITCH, name: "dip_switch", pins: 4, pins_max: 16, params: 9, blob: BlobShape::Exact(0), check: dip_switch_check, live_params: &[1, 2, 3, 4, 5, 6, 7, 8], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: LED, name: "led", pins: 2, pins_max: 2, params: led_param::COUNT, blob: BlobShape::Exact(0), check: led_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: RGB_LED, name: "rgb_led", pins: 4, pins_max: 4, params: rgb_param::COUNT, blob: BlobShape::Exact(1), check: rgb_led_check, live_params: &[], digital_slots: no_slots, channel_slots: rgb_channels },
    KindShape { code: BICOLOR_LED, name: "bicolor_led", pins: 3, pins_max: 3, params: bicolor_param::COUNT, blob: BlobShape::Exact(1), check: bicolor_led_check, live_params: &[], digital_slots: no_slots, channel_slots: bicolor_channels },
    KindShape { code: SEG7_CC, name: "seg7_cc", pins: 10, pins_max: 10, params: seg7_param::COUNT, blob: BlobShape::Exact(1), check: no_checks, live_params: &[], digital_slots: no_slots, channel_slots: seg7_channels },
    KindShape { code: SEG7_CA, name: "seg7_ca", pins: 10, pins_max: 10, params: seg7_param::COUNT, blob: BlobShape::Exact(1), check: no_checks, live_params: &[], digital_slots: no_slots, channel_slots: seg7_channels },
    KindShape { code: NE555, name: "ne555", pins: 2 + NE555_LOGIC_PINS, pins_max: 2 + NE555_LOGIC_PINS, params: ic_param::COUNT, blob: BlobShape::Exact(1 + NE555_LOGIC_PINS), check: ne555_check, live_params: &[], digital_slots: one_slot, channel_slots: no_slots },
    ic_shape!(IC_74LS00, "74ls00", 12, 4),
    ic_shape!(IC_74LS04, "74ls04", 12, 6),
    ic_shape!(IC_74LS08, "74ls08", 12, 4),
    ic_shape!(IC_74LS32, "74ls32", 12, 4),
    ic_shape!(IC_74LS86, "74ls86", 12, 4),
    ic_shape!(IC_74LS157, "74ls157", 14, 4, 1, ic_tail_check),
    ic_shape!(IC_74LS283, "74ls283", 14, 5),
    ic_shape!(IC_74HC138, "74hc138", 14, 8),
    ic_shape!(IC_74HC14, "74hc14", 12, 6),
    ic_shape!(IC_74LS47, "74ls47", 14, 7),
    ic_shape!(IC_74LS245, "74ls245", 18, 16, 2, ic_tail_check),
    w1d_shape!(IC_74LS161, "74ls161", 14, 21, 1 + 14, 5, seq_check),
    w1d_shape!(IC_74LS173, "74ls173", 14, 21, 1 + 14, 4, seq_check),
    w1d_shape!(IC_74HC74, "74hc74", 12, 17, 1 + 12, 4, seq_check),
    w1d_shape!(IC_74LS189, "74ls189", 14, 23, 1 + 14, 4, seq_check),
    w1d_shape!(IC_28C16, "28c16", 22, eeprom_param::COUNT, 1 + 22 + 2048 / 4, 8, eeprom_28c16_check),
    w1d_shape!(IC_28C256, "28c256", 26, eeprom_param::COUNT, 1 + 26 + 32768 / 4, 8, eeprom_check),
];

/// The shape for `code`, or `None` when the code is unknown or reserved.
pub fn shape(code: u32) -> Option<&'static KindShape> {
    SHAPES.iter().find(|s| s.code == code)
}
