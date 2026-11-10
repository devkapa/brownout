//! W2c: load carry-forward (core/README.md, "Carry"; W2 design 2.1, rulings
//! R57 D8 (a) and D9 (a)).
//!
//! **Take.** `bo_engine_carry_take` collects, per device, a versioned record
//! of the state a reload carries the way classic's `load()` does
//! (SE:2807-3200): capacitor voltages, IC registers, memories' write cycles,
//! the 555's latch, edge history, driver commits in flight, battery SoC, a
//! PSU's regime, and the slow physics (stress, temperature, PTC, latched
//! failures). The TS side maps the old records onto the new program's devices
//! by classic's rules (id, then net-pair fingerprint, kind-gated) and writes
//! them into the program's `CARRY` block (program v4), which `bo_engine_load`
//! seeds before its settle.
//!
//! **Records.** The buffer is a run of records, each
//! `[device, family, version, len, value x len]`. A family's layout only
//! grows: a reader takes the first values it knows and ignores the rest, and
//! a version it does not know reads as none. The engine is between calls when
//! a carry is taken, so every value is committed state at `bo_engine_time`;
//! times in a record (a commit's due time, a write's completion) are
//! absolute.

/// A capacitor: `[vc]`, the charge's voltage (the internal node behind any
/// esr), at the engine's time.
pub const FAMILY_CAPACITOR: u32 = 1;
/// An IC (see [`ic`] for the layout).
pub const FAMILY_IC: u32 = 2;
/// A battery pack: `bo_engine_device_state`'s battery layout (`soc`, `ocv`,
/// `r`, remaining and discharged coulombs, rejected recharge).
pub const FAMILY_BATTERY: u32 = 3;
/// A bench PSU with a current limit: `[reg]` (0 CV, 2 CC).
pub const FAMILY_PSU: u32 = 4;
/// A device's slow physics (see [`slow`] for the layout).
pub const FAMILY_SLOW: u32 = 5;
/// One latched failure: `bo_engine_records`' record less its device,
/// `[code, pin, since, value, limit, aux]`. A device may have several (an
/// IC's `output_sag`, one per pin).
pub const FAMILY_LATCH: u32 = 6;

/// Every family's current layout version.
pub const VERSION: u32 = 1;

/// The words before a record's values.
pub const HEADER: usize = 4;

/// Appends one record.
pub fn push(out: &mut Vec<f64>, device: usize, family: u32, values: &[f64]) {
    out.extend([device as f64, family as f64, VERSION as f64, values.len() as f64]);
    out.extend_from_slice(values);
}

/// One record read back.
#[derive(Debug, Clone, PartialEq)]
pub struct Record<'a> {
    pub device: usize,
    pub family: u32,
    pub version: u32,
    pub values: &'a [f64],
}

/// Splits a carry buffer into its records, or says where it is malformed.
pub fn records(buf: &[f64]) -> Result<Vec<Record<'_>>, String> {
    let mut out = Vec::new();
    let mut at = 0;
    while at < buf.len() {
        let head = buf.get(at..at + HEADER).ok_or_else(|| format!("carry: a record header cut short at word {at}"))?;
        let whole = |v: f64, what: &str| -> Result<usize, String> {
            if v.is_finite() && v >= 0.0 && v.fract() == 0.0 && v <= u32::MAX as f64 {
                Ok(v as usize)
            } else {
                Err(format!("carry: {what} {v} at word {at} is not a whole number"))
            }
        };
        let (device, family, version, len) = (whole(head[0], "device")?, whole(head[1], "family")?, whole(head[2], "version")?, whole(head[3], "length")?);
        let values = buf.get(at + HEADER..at + HEADER + len).ok_or_else(|| format!("carry: the record at word {at} runs past the end"))?;
        out.push(Record { device, family: family as u32, version: version as u32, values });
        at += HEADER + len;
    }
    Ok(out)
}

/// Encodings shared by the IC layout.
pub mod ic {
    use crate::events::Mode;

    /// The IC family tag, the record's second value (after the kind code).
    pub const COMBINATIONAL: f64 = 0.0;
    pub const SEQUENTIAL: f64 = 1.0;
    pub const RAM_189: f64 = 2.0;
    pub const EEPROM: f64 = 3.0;
    pub const NE555: f64 = 4.0;

    /// A driver mode: `-1` none, `0` Hi-Z, `1` high, `2` low.
    pub fn mode(m: Option<Mode>) -> f64 {
        match m {
            None => -1.0,
            Some(Mode::Hiz) => 0.0,
            Some(Mode::High) => 1.0,
            Some(Mode::Low) => 2.0,
        }
    }

    pub fn bit(b: bool) -> f64 {
        f64::from(u8::from(b))
    }

    /// FNV-1a over a memory image, as an f64-exact 32-bit hash: classic
    /// carries an EEPROM only onto the same contents (SE: by contents hash),
    /// and the image itself reloads from the program's ROM blob.
    pub fn image_hash(bytes: &[u8]) -> f64 {
        let mut h: u32 = 0x811c_9dc5;
        for &b in bytes {
            h ^= u32::from(b);
            h = h.wrapping_mul(0x0100_0193);
        }
        f64::from(h)
    }
}

/// The slow layout: `[failed, stress, temperature, shutdown, ptc_tripped,
/// ptc_stress, ptc_rt]`, at the engine's time. `NaN` for a quantity the
/// device has none of (no stress integral, no thermal model, not a PTC);
/// `failed` is 0 or 1. Its latch records are their own family.
pub mod slow {
    pub const LEN: usize = 7;
}
