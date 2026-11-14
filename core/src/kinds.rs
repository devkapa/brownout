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
pub const PTC_FUSE: u32 = 13;

pub const BATTERY_PACK: u32 = 20;
pub const BENCH_PSU: u32 = 21;
pub const CLOCK: u32 = 22;
pub const CLOCK_GEN: u32 = 23;
pub const PULSE_SOURCE: u32 = 24;
pub const PULSE_GEN: u32 = 25;
/// Synthetic: the compiler emits one per isolated section, never a component
/// kind (core/README.md, "Section anchor record").
pub const SECTION_ANCHOR: u32 = 26;

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
// W1g's synthetic records (core/README.md, "Rating record" and "Thermal
// record"): no pins, published nothing, read only by the slow state.
pub const RATING: u32 = 90;
pub const THERMAL: u32 = 91;

/// How many blob values a kind's record carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlobShape {
    Exact(usize),
    /// One of two exact lengths: a record with an optional tail (the
    /// 74ls189's preload), absent or whole. A program written before the
    /// tail existed has the first length and still loads (ruling R100's
    /// rule for an addition).
    Either(usize, usize),
    /// Variable length with a floor, for contents such as ROM bytes.
    AtLeast(usize),
    /// `blob[0] = n`, then `n` pairs: length exactly `1 + 2n` with `n >= 1`
    /// (the battery's OCV table). The reader checks this shape; the kind's
    /// `check` checks the values.
    Table,
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

/// Words in a 74ls189's optional preload, which follows its pin flags in the
/// blob (core/README.md, "RAM preload"): one value per address, each the
/// nibble stored there in true sense. 0 for any other kind.
pub fn ram_preload_words(kind: u32) -> usize {
    match kind {
        IC_74LS189 => 16,
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

/// The 74ls189 (code 80) on top of [`seq_check`]: every preload value after
/// the pin flags (none, or sixteen; the reader checks the count) is an
/// integer in `[0, 15]`.
fn ram_189_check(pin_count: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    seq_check(pin_count, params, blob)?;
    let words_from = 1 + pin_count.saturating_sub(2);
    if let Some(bad) = blob.get(words_from..).unwrap_or(&[]).iter().find(|v| !is_code_in(**v, 15)) {
        return Err(format!("preload words must be integers in [0, 15], got {bad}"));
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

/// `fuse` params (core/README.md, "Fuse and PTC records"): `[rNormal,
/// iRating]`. The compiler floors `rNormal` at 0.001, so the stamp's
/// `1/rNormal` is finite; `iRating <= 0` is legal and means no limit.
pub mod fuse_param {
    pub const R_NORMAL: usize = 0;
    pub const I_RATING: usize = 1;
    pub const COUNT: usize = 2;
}

/// `ptc_fuse` params: `[rNormal, iHold]`, both floored by the compiler.
pub mod ptc_param {
    pub const R_NORMAL: usize = 0;
    pub const I_HOLD: usize = 1;
    pub const COUNT: usize = 2;
}

/// The compiler's floors (PA:694, 755, 774), repeated so a hand-built program
/// cannot divide by zero in a stamp or a trip threshold.
pub const FUSE_MIN_R_NORMAL: f64 = 0.001;
pub const PTC_MIN_I_HOLD: f64 = 1e-6;

fn fuse_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !(params[fuse_param::R_NORMAL] >= FUSE_MIN_R_NORMAL) {
        return Err(format!("fuse rNormal must be >= {FUSE_MIN_R_NORMAL}, got {}", params[fuse_param::R_NORMAL]));
    }
    Ok(())
}

fn ptc_fuse_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !(params[ptc_param::R_NORMAL] >= FUSE_MIN_R_NORMAL) {
        return Err(format!("ptc_fuse rNormal must be >= {FUSE_MIN_R_NORMAL}, got {}", params[ptc_param::R_NORMAL]));
    }
    if !(params[ptc_param::I_HOLD] >= PTC_MIN_I_HOLD) {
        return Err(format!("ptc_fuse iHold must be >= {PTC_MIN_I_HOLD:e}, got {}", params[ptc_param::I_HOLD]));
    }
    Ok(())
}

/// `rating` params (code 90): `[device, pMax, rPower]`. `device` is checked
/// against the other records by `program.rs` (it spans records); here only
/// that it is an index at all.
pub mod rating_param {
    pub const DEVICE: usize = 0;
    pub const P_MAX: usize = 1;
    pub const R_POWER: usize = 2;
    pub const COUNT: usize = 3;
}

/// `thermal` params (code 91): `[device, rThetaJA, tauS, hasShutdown, tripC,
/// restartC]`.
pub mod thermal_param {
    pub const DEVICE: usize = 0;
    pub const R_THETA: usize = 1;
    pub const TAU_S: usize = 2;
    pub const HAS_SHUTDOWN: usize = 3;
    pub const TRIP_C: usize = 4;
    pub const RESTART_C: usize = 5;
    pub const COUNT: usize = 6;
}

fn device_ref_check(device: f64) -> Result<(), String> {
    if device >= 0.0 && device.fract() == 0.0 && device <= u32::MAX as f64 {
        Ok(())
    } else {
        Err(format!("device must be a record index, got {device}"))
    }
}

fn rating_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    device_ref_check(params[rating_param::DEVICE])
}

fn thermal_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    use thermal_param::*;
    device_ref_check(params[DEVICE])?;
    if !(params[R_THETA] > 0.0) {
        return Err(format!("thermal rThetaJA must be > 0, got {}", params[R_THETA]));
    }
    if !(params[TAU_S] > 0.0) {
        return Err(format!("thermal tauS must be > 0, got {}", params[TAU_S]));
    }
    match params[HAS_SHUTDOWN] {
        0.0 => {
            if params[TRIP_C] != 0.0 || params[RESTART_C] != 0.0 {
                return Err(format!("thermal tripC and restartC must be 0 without a shutdown, got {} and {}", params[TRIP_C], params[RESTART_C]));
            }
        }
        1.0 => {
            if !(params[RESTART_C] < params[TRIP_C]) {
                return Err(format!("thermal restartC must be below tripC, got {} and {}", params[RESTART_C], params[TRIP_C]));
            }
        }
        other => return Err(format!("thermal hasShutdown must be 0 or 1, got {other}")),
    }
    Ok(())
}

/// `battery_pack` params (core/README.md, "Battery record"). The last is
/// the profile code, which Rust ignores and the facade reads back.
pub mod battery_param {
    pub const OCV_SCALE: usize = 0;
    pub const R_BASE: usize = 1;
    pub const TEMP_FACTOR: usize = 2;
    pub const DEP_A: usize = 3;
    pub const DEP_P: usize = 4;
    pub const DEP_K: usize = 5;
    pub const CAPACITY_AH: usize = 6;
    pub const SOC0: usize = 7;
    pub const PROFILE_CODE: usize = 8;
    pub const COUNT: usize = 9;
}

/// The smallest capacity the compiler emits (`max(1e-12, ...)`), which keeps
/// `capacityAh * 3600` a divisor.
pub const BATTERY_MIN_CAPACITY_AH: f64 = 1e-12;
/// Profile codes: 0 generic, 1 alkaline-aa-4s, 2 alkaline-pp3-9v, 3
/// lithium-cr2032.
const BATTERY_MAX_PROFILE_CODE: u32 = 3;

/// The values of a `battery_pack` record on top of the shape check (core/README.md,
/// "Battery record", Validation). Written to stand alone: it does not lean on
/// the reader having checked the blob's length.
///
/// `depP >= 0` is not in the contract's list. A negative exponent makes
/// `(1 - soc)^depP` infinite at full charge, so `R` would be infinite; no
/// profile has one.
fn battery_check(_: usize, params: &[f64], blob: &[f64]) -> Result<(), String> {
    use battery_param::*;
    if !(params[OCV_SCALE] >= 0.0) {
        return Err(format!("battery ocvScale must be >= 0, got {}", params[OCV_SCALE]));
    }
    if !(params[R_BASE] >= 0.0) {
        return Err(format!("battery rBase must be >= 0, got {}", params[R_BASE]));
    }
    if !(params[TEMP_FACTOR] > 0.0) {
        return Err(format!("battery tempFactor must be > 0, got {}", params[TEMP_FACTOR]));
    }
    if !(params[DEP_A] >= 0.0) {
        return Err(format!("battery depA must be >= 0, got {}", params[DEP_A]));
    }
    if !(params[DEP_P] >= 0.0) {
        return Err(format!("battery depP must be >= 0, got {}", params[DEP_P]));
    }
    if !(params[DEP_K] > 0.0) {
        return Err(format!("battery depK must be > 0, got {}", params[DEP_K]));
    }
    if !(params[CAPACITY_AH] >= BATTERY_MIN_CAPACITY_AH) {
        return Err(format!("battery capacityAh must be >= {BATTERY_MIN_CAPACITY_AH:e}, got {}", params[CAPACITY_AH]));
    }
    if !(0.0..=1.0).contains(&params[SOC0]) {
        return Err(format!("battery soc0 must be in [0, 1], got {}", params[SOC0]));
    }
    if !is_code_in(params[PROFILE_CODE], BATTERY_MAX_PROFILE_CODE) {
        return Err(format!("battery profileCode must be an integer in [0, {BATTERY_MAX_PROFILE_CODE}], got {}", params[PROFILE_CODE]));
    }
    let n = blob.first().copied().ok_or_else(|| "battery OCV table is empty".to_string())?;
    if !(n >= 1.0 && n.fract() == 0.0) {
        return Err(format!("battery OCV table point count must be a positive integer, got {n}"));
    }
    if (blob.len() - 1) as f64 != 2.0 * n {
        return Err(format!("battery OCV table of {n} point(s) needs {} blob value(s), got {}", 1.0 + 2.0 * n, blob.len()));
    }
    let mut previous = 0.0;
    for (i, pair) in blob[1..].chunks_exact(2).enumerate() {
        let soc = pair[0];
        if !(0.0..=1.0).contains(&soc) {
            return Err(format!("battery OCV table soc_{i} must be in [0, 1], got {soc}"));
        }
        if soc < previous {
            return Err(format!("battery OCV table soc_{i} = {soc} is below soc_{} = {previous}", i.saturating_sub(1)));
        }
        previous = soc;
    }
    Ok(())
}

/// `bench_psu` params `[V, iLimit]`. `iLimit` is 0 for an ideal source and
/// positive for a current-limited one; the compiler emits 0 for anything
/// that is not `> 0`, so a negative value has no encoding here.
fn bench_psu_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !(params[1] >= 0.0) {
        return Err(format!("bench_psu iLimit must be >= 0, got {}", params[1]));
    }
    Ok(())
}

/// `clock`, `clock_gen` params `[freq, duty, vHigh, vLow, delay]`: the
/// compiler emits `max(1e-6, frequency)`, so the period `1 / freq` is finite.
pub const CLOCK_MIN_FREQ: f64 = 1e-6;

fn clock_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    if !(params[0] >= CLOCK_MIN_FREQ) {
        return Err(format!("clock freq must be >= {CLOCK_MIN_FREQ:e}, got {}", params[0]));
    }
    Ok(())
}

/// `pulse_source`, `pulse_gen` params `[v1, v2, td, tr, tf, pw, per]`
/// (core/README.md, "Waveform source records"): the compiler's declines,
/// repeated. `per` is 0 for a one-shot, so a negative one has no encoding.
fn pulse_check(_: usize, params: &[f64], _: &[f64]) -> Result<(), String> {
    let (tr, tf, pw, per) = (params[3], params[4], params[5], params[6]);
    for (name, v) in [("tr", tr), ("tf", tf), ("pw", pw), ("per", per)] {
        if !(v >= 0.0) {
            return Err(format!("pulse {name} must be >= 0, got {v}"));
        }
    }
    if per > 0.0 && per < tr + pw + tf {
        return Err(format!("truncated pulse: per {per} < tr + pw + tf = {}", tr + pw + tf));
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
    ($code:expr, $name:expr, $logic:expr, $params:expr, $blob:expr, $slots:expr, $check:expr) => {
        w1d_shape!($code, $name, $logic, $params, BlobShape::Exact($blob), $slots, $check, shaped)
    };
    ($code:expr, $name:expr, $logic:expr, $params:expr, $blob:expr, $slots:expr, $check:expr, shaped) => {{
        fn slots(_: usize) -> usize {
            $slots
        }
        KindShape {
            code: $code,
            name: $name,
            pins: 2 + $logic,
            pins_max: 2 + $logic,
            params: $params,
            blob: $blob,
            check: $check,
            live_params: &[],
            digital_slots: slots,
            channel_slots: no_slots,
        }
    }};
}

const SHAPES: &[KindShape] = &[
    KindShape { code: RESISTOR, name: "resistor", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: resistor_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: VOLTAGE_SOURCE, name: "voltage_source", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: no_checks, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: CAPACITOR, name: "capacitor", pins: 2, pins_max: 2, params: 3, blob: BlobShape::Exact(0), check: capacitor_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: SWITCH, name: "switch", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: PUSH_BUTTON, name: "push_button", pins: 2, pins_max: 2, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: SPDT_SWITCH, name: "spdt_switch", pins: 3, pins_max: 3, params: 1, blob: BlobShape::Exact(0), check: single_switch_check, live_params: &[0], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: FUSE, name: "fuse", pins: 2, pins_max: 2, params: fuse_param::COUNT, blob: BlobShape::Exact(0), check: fuse_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: PTC_FUSE, name: "ptc_fuse", pins: 2, pins_max: 2, params: ptc_param::COUNT, blob: BlobShape::Exact(0), check: ptc_fuse_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: DIP_SWITCH, name: "dip_switch", pins: 4, pins_max: 16, params: 9, blob: BlobShape::Exact(0), check: dip_switch_check, live_params: &[1, 2, 3, 4, 5, 6, 7, 8], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: BATTERY_PACK, name: "battery_pack", pins: 2, pins_max: 2, params: battery_param::COUNT, blob: BlobShape::Table, check: battery_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: BENCH_PSU, name: "bench_psu", pins: 2, pins_max: 2, params: 2, blob: BlobShape::Exact(0), check: bench_psu_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: CLOCK, name: "clock", pins: 2, pins_max: 2, params: 5, blob: BlobShape::Exact(1), check: clock_check, live_params: &[], digital_slots: one_slot, channel_slots: no_slots },
    KindShape { code: CLOCK_GEN, name: "clock_gen", pins: 2, pins_max: 2, params: 5, blob: BlobShape::Exact(1), check: clock_check, live_params: &[], digital_slots: one_slot, channel_slots: no_slots },
    KindShape { code: PULSE_SOURCE, name: "pulse_source", pins: 2, pins_max: 2, params: 7, blob: BlobShape::Exact(1), check: pulse_check, live_params: &[], digital_slots: one_slot, channel_slots: no_slots },
    KindShape { code: PULSE_GEN, name: "pulse_gen", pins: 2, pins_max: 2, params: 7, blob: BlobShape::Exact(1), check: pulse_check, live_params: &[], digital_slots: one_slot, channel_slots: no_slots },
    KindShape { code: SECTION_ANCHOR, name: "section_anchor", pins: 1, pins_max: 1, params: 0, blob: BlobShape::Exact(0), check: no_checks, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
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
    w1d_shape!(IC_74LS189, "74ls189", 14, 23, BlobShape::Either(1 + 14, 1 + 14 + 16), 4, ram_189_check, shaped),
    w1d_shape!(IC_28C16, "28c16", 22, eeprom_param::COUNT, 1 + 22 + 2048 / 4, 8, eeprom_28c16_check),
    w1d_shape!(IC_28C256, "28c256", 26, eeprom_param::COUNT, 1 + 26 + 32768 / 4, 8, eeprom_check),
    KindShape { code: RATING, name: "rating", pins: 0, pins_max: 0, params: rating_param::COUNT, blob: BlobShape::Exact(0), check: rating_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
    KindShape { code: THERMAL, name: "thermal", pins: 0, pins_max: 0, params: thermal_param::COUNT, blob: BlobShape::Exact(0), check: thermal_check, live_params: &[], digital_slots: no_slots, channel_slots: no_slots },
];

/// Whether `kind` is one of W1g's synthetic records (codes 90 and 91), which
/// stamp and publish nothing.
pub fn is_synthetic_slow(kind: u32) -> bool {
    matches!(kind, RATING | THERMAL)
}

/// The shape for `code`, or `None` when the code is unknown or reserved.
pub fn shape(code: u32) -> Option<&'static KindShape> {
    SHAPES.iter().find(|s| s.code == code)
}

#[cfg(test)]
mod tests {
    //! The W1f source records (codes 20-26) through the program reader: what
    //! it accepts, and that each malformed record is refused as `-1` by the
    //! rule it breaks, not by some other one (every rejection asserts its
    //! message names the field).
    use super::*;
    use crate::error::ERR_MALFORMED;
    use crate::program::{Program, MAGIC, VERSION_3};

    /// `[ocvScale, rBase, tempFactor, depA, depP, depK, capacityAh, soc0,
    /// profileCode]`: a `{5 V, 0.9 ohm}` generic battery.
    fn battery_params() -> Vec<f64> {
        vec![1.0, 0.9, 1.0, 0.5, 2.0, 0.05, 1.0, 1.0, 0.0]
    }

    fn generic_table() -> Vec<f64> {
        vec![2.0, 0.0, 0.0, 1.0, 5.0]
    }

    fn record(kind: u32, nodes: &[f64], params: &[f64], blob: &[f64]) -> Vec<f64> {
        let mut out = vec![kind as f64, 0.0, nodes.len() as f64];
        for &n in nodes {
            out.extend([n, 0.0, 0.0, 0.0]);
        }
        out.push(params.len() as f64);
        out.extend_from_slice(params);
        out.push(blob.len() as f64);
        out.extend_from_slice(blob);
        out
    }

    fn program(digital_slots: f64, devices: &[Vec<f64>]) -> Vec<f64> {
        let mut out = vec![MAGIC, VERSION_3, 4.0, devices.len() as f64, 0.0, digital_slots, 0.0];
        for d in devices {
            out.extend_from_slice(d);
        }
        out.push(MAGIC);
        out
    }

    fn parse_one(digital_slots: f64, device: Vec<f64>) -> Result<Program, crate::error::BoError> {
        Program::parse(&program(digital_slots, &[device]))
    }

    /// Asserts the record is refused as `-1` and the message names `field`.
    fn assert_refused(digital_slots: f64, device: Vec<f64>, field: &str) {
        let err = parse_one(digital_slots, device).expect_err(&format!("a record with a bad {field} must be refused"));
        assert_eq!(err.code, ERR_MALFORMED, "{field}: {}", err.message);
        assert!(err.message.contains(field), "the message should name `{field}`: {}", err.message);
    }

    fn battery_with(edit: impl FnOnce(&mut Vec<f64>)) -> Vec<f64> {
        let mut params = battery_params();
        edit(&mut params);
        record(BATTERY_PACK, &[1.0, 0.0], &params, &generic_table())
    }

    fn battery_with_table(blob: &[f64]) -> Vec<f64> {
        record(BATTERY_PACK, &[1.0, 0.0], &battery_params(), blob)
    }

    fn psu(v: f64, i_limit: f64) -> Vec<f64> {
        record(BENCH_PSU, &[1.0, 0.0], &[v, i_limit], &[])
    }

    fn clock_record(kind: u32, freq: f64, base: f64) -> Vec<f64> {
        record(kind, &[1.0, 0.0], &[freq, 0.5, 5.0, 0.0, 0.0], &[base])
    }

    fn pulse_record(kind: u32, params: [f64; 7], base: f64) -> Vec<f64> {
        record(kind, &[1.0, 0.0], &params, &[base])
    }

    const CATALOG_PULSE: [f64; 7] = [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 1e-3];

    // ---- what is accepted -------------------------------------------------

    #[test]
    fn every_well_formed_source_record_parses_and_claims_its_slots() {
        let devices = [
            record(BATTERY_PACK, &[1.0, 0.0], &battery_params(), &generic_table()),
            psu(5.0, 0.0),
            psu(-5.0, 0.1),
            clock_record(CLOCK, 1000.0, 0.0),
            clock_record(CLOCK_GEN, 1000.0, 1.0),
            pulse_record(PULSE_SOURCE, CATALOG_PULSE, 2.0),
            pulse_record(PULSE_GEN, CATALOG_PULSE, 3.0),
            record(SECTION_ANCHOR, &[2.0], &[], &[]),
        ];
        let p = Program::parse(&program(4.0, &devices)).expect("all seven kinds parse together");
        let kinds: Vec<u32> = p.devices.iter().map(|d| d.kind).collect();
        assert_eq!(kinds, [BATTERY_PACK, BENCH_PSU, BENCH_PSU, CLOCK, CLOCK_GEN, PULSE_SOURCE, PULSE_GEN, SECTION_ANCHOR]);
        assert_eq!(p.devices[0].blob, generic_table());
        assert_eq!(p.devices[7].pins.len(), 1);
        assert_eq!(p.devices[7].pins[0].node, 2);
    }

    #[test]
    fn the_shapes_match_the_readme_table() {
        for (code, name, pins, params) in [
            (BATTERY_PACK, "battery_pack", 2, 9),
            (BENCH_PSU, "bench_psu", 2, 2),
            (CLOCK, "clock", 2, 5),
            (CLOCK_GEN, "clock_gen", 2, 5),
            (PULSE_SOURCE, "pulse_source", 2, 7),
            (PULSE_GEN, "pulse_gen", 2, 7),
            (SECTION_ANCHOR, "section_anchor", 1, 0),
        ] {
            let s = shape(code).unwrap_or_else(|| panic!("{name} has a shape"));
            assert_eq!((s.name, s.pins, s.pins_max, s.params), (name, pins, pins, params));
            assert!(s.live_params.is_empty(), "{name}: no live params");
            assert_eq!((s.channel_slots)(pins), 0, "{name}: no channel slots");
            let digital = if matches!(code, CLOCK | CLOCK_GEN | PULSE_SOURCE | PULSE_GEN) { 1 } else { 0 };
            assert_eq!((s.digital_slots)(pins), digital, "{name}: digital slots");
        }
        assert_eq!(shape(BATTERY_PACK).unwrap().blob, BlobShape::Table);
        assert_eq!(shape(BENCH_PSU).unwrap().blob, BlobShape::Exact(0));
        assert_eq!(shape(CLOCK).unwrap().blob, BlobShape::Exact(1));
        assert_eq!(shape(PULSE_GEN).unwrap().blob, BlobShape::Exact(1));
        assert_eq!(shape(SECTION_ANCHOR).unwrap().blob, BlobShape::Exact(0));
        assert_eq!(SECTION_ANCHOR, 26);
    }

    #[test]
    fn an_unassigned_code_is_unknown_kind() {
        // 9999 is far above every planned kind (the table runs in decade
        // bands, 1-12, 20-26, 30-34, 40, 50-61, 70-76, 80-82), so no later
        // wave assigns it and this check outlives them. A code a wave does
        // assign must not be pinned here as "reserved".
        const FAR_UNASSIGNED: u32 = 9999;
        assert!(shape(FAR_UNASSIGNED).is_none());
        let err = parse_one(0.0, record(FAR_UNASSIGNED, &[1.0], &[], &[])).unwrap_err();
        assert_eq!(err.code, crate::error::ERR_UNKNOWN_KIND);
    }

    // ---- battery ---------------------------------------------------------------

    #[test]
    fn a_battery_with_every_value_at_its_bound_is_accepted() {
        // ocvScale 0, rBase 0, depA 0, depP 0, capacity at its floor, soc0 at
        // both ends, the last profile code, a one-point table.
        for soc0 in [0.0, 1.0] {
            let d = battery_with(|p| {
                *p = vec![0.0, 0.0, f64::MIN_POSITIVE, 0.0, 0.0, f64::MIN_POSITIVE, BATTERY_MIN_CAPACITY_AH, soc0, 3.0];
            });
            parse_one(0.0, d).unwrap_or_else(|e| panic!("soc0 {soc0}: {}", e.message));
        }
        parse_one(0.0, battery_with_table(&[1.0, 0.5, 3.3])).expect("one point");
        parse_one(0.0, battery_with_table(&[3.0, 0.0, 0.0, 0.5, 1.0, 0.5, 2.0])).expect("a repeated soc is a step, not a violation");
        parse_one(0.0, battery_with_table(&[2.0, 0.0, -1.0, 1.0, 4.0])).expect("negative volts are the table's business");
    }

    #[test]
    fn a_battery_refuses_each_bad_param() {
        use battery_param::*;
        for (index, bad, field) in [
            (OCV_SCALE, -0.1, "ocvScale"),
            (R_BASE, -1e-9, "rBase"),
            (TEMP_FACTOR, 0.0, "tempFactor"),
            (TEMP_FACTOR, -1.0, "tempFactor"),
            (DEP_A, -0.5, "depA"),
            (DEP_P, -2.0, "depP"),
            (DEP_K, 0.0, "depK"),
            (DEP_K, -0.05, "depK"),
            (CAPACITY_AH, 0.0, "capacityAh"),
            (CAPACITY_AH, 1e-13, "capacityAh"),
            (SOC0, -0.01, "soc0"),
            (SOC0, 1.01, "soc0"),
            (PROFILE_CODE, 4.0, "profileCode"),
            (PROFILE_CODE, -1.0, "profileCode"),
            (PROFILE_CODE, 1.5, "profileCode"),
        ] {
            assert_refused(0.0, battery_with(|p| p[index] = bad), field);
        }
    }

    #[test]
    fn a_battery_refuses_a_non_finite_param_and_a_wrong_param_count() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_refused(0.0, battery_with(|p| p[battery_param::R_BASE] = bad), "not finite");
        }
        let mut short = battery_params();
        short.pop();
        assert_refused(0.0, record(BATTERY_PACK, &[1.0, 0.0], &short, &generic_table()), "param");
    }

    #[test]
    fn a_battery_refuses_each_bad_table() {
        assert_refused(0.0, battery_with_table(&[]), "blob");
        assert_refused(0.0, battery_with_table(&[0.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[-1.0, 0.0, 0.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[1.5, 0.0, 0.0, 1.0, 5.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[2.0, 0.0, 0.0, 1.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[2.0, 0.0, 0.0, 1.0, 5.0, 9.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[3.0, 0.0, 0.0, 1.0, 5.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[1e300, 0.0, 0.0]), "n >= 1");
        assert_refused(0.0, battery_with_table(&[1.0, 0.0, 0.0, 9.0]), "n >= 1");
        // Values: soc outside [0, 1] and a table that goes back down.
        assert_refused(0.0, battery_with_table(&[2.0, -0.1, 0.0, 1.0, 5.0]), "soc_0");
        assert_refused(0.0, battery_with_table(&[2.0, 0.0, 0.0, 1.1, 5.0]), "soc_1");
        assert_refused(0.0, battery_with_table(&[3.0, 0.0, 0.0, 0.6, 3.0, 0.5, 4.0]), "below");
    }

    #[test]
    fn battery_check_stands_alone_on_a_table_the_reader_never_shaped() {
        // The reader's `Table` shape check rejects these first; `check` is also
        // reachable without it and must not index past the blob.
        let params = battery_params();
        for blob in [&[][..], &[0.0], &[-2.0, 0.0, 0.0], &[1.5, 0.0, 0.0, 1.0, 5.0], &[2.0, 0.0, 0.0], &[1.0, 0.0, 0.0, 1.0, 5.0], &[1e300, 0.0, 0.0]] {
            let err = battery_check(2, &params, blob).expect_err(&format!("{blob:?}"));
            assert!(err.contains("OCV table"), "{blob:?}: {err}");
        }
        assert!(battery_check(2, &params, &generic_table()).is_ok());
    }

    #[test]
    fn a_battery_refuses_the_wrong_pin_count() {
        assert_refused(0.0, record(BATTERY_PACK, &[1.0], &battery_params(), &generic_table()), "pin");
        assert_refused(0.0, record(BATTERY_PACK, &[1.0, 0.0, 2.0], &battery_params(), &generic_table()), "pin");
    }

    // ---- bench psu ---------------------------------------------------------------

    #[test]
    fn a_bench_psu_accepts_both_forms_and_refuses_a_negative_limit() {
        parse_one(0.0, psu(5.0, 0.0)).expect("ideal");
        parse_one(0.0, psu(5.0, 2.0)).expect("limited");
        parse_one(0.0, psu(-12.0, 0.5)).expect("a negative setpoint");
        parse_one(0.0, psu(0.0, 0.5)).expect("a zero setpoint");
        assert_refused(0.0, psu(5.0, -0.1), "iLimit");
        assert_refused(0.0, psu(5.0, -f64::MIN_POSITIVE), "iLimit");
        assert_refused(0.0, psu(f64::NAN, 0.1), "not finite");
        assert_refused(0.0, psu(5.0, f64::INFINITY), "not finite");
    }

    #[test]
    fn a_bench_psu_carries_no_blob_and_two_pins() {
        assert_refused(0.0, record(BENCH_PSU, &[1.0, 0.0], &[5.0, 0.1], &[0.0]), "blob");
        assert_refused(0.0, record(BENCH_PSU, &[1.0], &[5.0, 0.1], &[]), "pin");
        assert_refused(0.0, record(BENCH_PSU, &[1.0, 0.0], &[5.0], &[]), "param");
    }

    // ---- clocks and pulses ---------------------------------------------------------

    #[test]
    fn a_clock_needs_a_period_the_compiler_could_have_emitted() {
        for kind in [CLOCK, CLOCK_GEN] {
            parse_one(1.0, clock_record(kind, 1e-6, 0.0)).expect("the compiler's floor");
            parse_one(1.0, clock_record(kind, 1e9, 0.0)).expect("a fast clock");
            assert_refused(1.0, clock_record(kind, 0.0, 0.0), "freq");
            assert_refused(1.0, clock_record(kind, 9e-7, 0.0), "freq");
            assert_refused(1.0, clock_record(kind, -1000.0, 0.0), "freq");
        }
        // Duty and levels are free: duty outside [0, 1] is a valid always-on
        // or always-off source (classic's clockVoltage compares only).
        parse_one(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, 7.0, 5.0, 0.0, -1.0], &[0.0])).expect("duty 7, negative delay");
        parse_one(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, -2.0, 0.0, 5.0, 0.0], &[0.0])).expect("duty -2, inverted levels");
        assert_refused(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, f64::NAN, 5.0, 0.0, 0.0], &[0.0]), "not finite");
    }

    #[test]
    fn a_pulse_refuses_negative_ramps_widths_and_periods() {
        for kind in [PULSE_SOURCE, PULSE_GEN] {
            for (index, field) in [(3, "pulse tr must"), (4, "pulse tf must"), (5, "pulse pw must"), (6, "pulse per must")] {
                let mut p = CATALOG_PULSE;
                p[index] = -1e-6;
                assert_refused(1.0, pulse_record(kind, p, 0.0), field);
            }
            let mut nan = CATALOG_PULSE;
            nan[1] = f64::NAN;
            assert_refused(1.0, pulse_record(kind, nan, 0.0), "not finite");
        }
    }

    #[test]
    fn a_pulse_refuses_a_truncated_cycle_and_accepts_every_other_period() {
        // per < tr + pw + tf cuts the cycle short (WF:52-56).
        for kind in [PULSE_SOURCE, PULSE_GEN] {
            assert_refused(1.0, pulse_record(kind, [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 4e-4], 0.0), "truncated");
            assert_refused(1.0, pulse_record(kind, [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 5.01e-4], 0.0), "truncated");
            // Exactly tr + pw + tf is a full cycle with no gap, and per 0 is a
            // one-shot whatever the widths.
            let exact = 1e-6 + 5e-4 + 1e-6;
            parse_one(1.0, pulse_record(kind, [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, exact], 0.0)).expect("no gap");
            parse_one(1.0, pulse_record(kind, [0.0, 5.0, 0.0, 1e-6, 1e-6, 5e-4, 0.0], 0.0)).expect("one-shot");
            parse_one(1.0, pulse_record(kind, [1.0, 1.0, -1e-3, 0.0, 0.0, 0.0, 1e-3], 0.0)).expect("a negative td, zero widths, v1 = v2");
        }
    }

    #[test]
    fn a_waveform_source_owns_one_digital_slot_that_must_exist() {
        for device in [clock_record(CLOCK, 1000.0, 3.0), pulse_record(PULSE_SOURCE, CATALOG_PULSE, 3.0)] {
            parse_one(4.0, device.clone()).expect("slot 3 of 4");
            assert_refused(3.0, device.clone(), "outside S");
            assert_refused(0.0, device, "outside S");
        }
        assert_refused(4.0, clock_record(CLOCK, 1000.0, 1.5), "outside S");
        assert_refused(4.0, clock_record(CLOCK, 1000.0, -1.0), "outside S");
        // Two sources cannot share a slot.
        let err = Program::parse(&program(4.0, &[clock_record(CLOCK, 1000.0, 2.0), pulse_record(PULSE_GEN, CATALOG_PULSE, 2.0)])).unwrap_err();
        assert_eq!(err.code, ERR_MALFORMED);
    }

    #[test]
    fn a_waveform_source_carries_exactly_its_params_blob_and_pins() {
        assert_refused(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, 0.5, 5.0, 0.0], &[0.0]), "param");
        assert_refused(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, 0.5, 5.0, 0.0, 0.0], &[]), "blob");
        assert_refused(1.0, record(CLOCK, &[1.0, 0.0], &[1000.0, 0.5, 5.0, 0.0, 0.0], &[0.0, 0.0]), "blob");
        assert_refused(1.0, record(PULSE_SOURCE, &[1.0, 0.0], &CATALOG_PULSE[..6], &[0.0]), "param");
        assert_refused(1.0, record(PULSE_SOURCE, &[1.0], &CATALOG_PULSE, &[0.0]), "pin");
        assert_refused(1.0, record(CLOCK_GEN, &[1.0, 0.0, 2.0], &[1000.0, 0.5, 5.0, 0.0, 0.0], &[0.0]), "pin");
    }

    // ---- section anchor -----------------------------------------------------------------

    #[test]
    fn a_section_anchor_is_one_pin_and_nothing_else() {
        parse_one(0.0, record(SECTION_ANCHOR, &[3.0], &[], &[])).expect("the well-formed record");
        assert_refused(0.0, record(SECTION_ANCHOR, &[], &[], &[]), "pin");
        assert_refused(0.0, record(SECTION_ANCHOR, &[1.0, 2.0], &[], &[]), "pin");
        assert_refused(0.0, record(SECTION_ANCHOR, &[1.0], &[0.0], &[]), "param");
        assert_refused(0.0, record(SECTION_ANCHOR, &[1.0], &[], &[0.0]), "blob");
    }
}

