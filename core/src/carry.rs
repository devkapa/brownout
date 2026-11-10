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

    /// [`mode`]'s inverse; `Err` for any other value.
    pub fn mode_of(v: f64) -> Result<Option<Mode>, String> {
        Ok(match v {
            -1.0 => None,
            0.0 => Some(Mode::Hiz),
            1.0 => Some(Mode::High),
            2.0 => Some(Mode::Low),
            _ => return Err(format!("carry: {v} is no driver mode")),
        })
    }

    /// [`bit`]'s inverse; `Err` for anything but 0 or 1.
    pub fn bit_of(v: f64) -> Result<bool, String> {
        if v == 0.0 || v == 1.0 {
            Ok(v == 1.0)
        } else {
            Err(format!("carry: {v} is no bit"))
        }
    }

    /// The family tag a kind's record carries.
    pub fn tag_of(kind: u32) -> f64 {
        use crate::digital;
        if digital::is_sequential_ic(kind) {
            SEQUENTIAL
        } else if kind == crate::kinds::IC_74LS189 {
            RAM_189
        } else if digital::is_memory_ic(kind) {
            EEPROM
        } else if digital::is_ne555(kind) {
            NE555
        } else {
            COMBINATIONAL
        }
    }

    /// An IC record split into its parts, each checked as far as the record
    /// alone can be (the counts against the IC itself are the seed's).
    #[derive(Debug, Clone, PartialEq)]
    pub struct Seed<'a> {
        /// The family values after the tag (a combinational IC's count
        /// included).
        pub family: &'a [f64],
        /// The logic-pin reads at the last powered evaluation, if any.
        pub reads: Option<Vec<bool>>,
        pub slots: &'a [f64],
        /// Per output driver: the committed mode and the pending `(target,
        /// due)`.
        pub drivers: Vec<(Option<Mode>, Option<(Mode, f64)>)>,
    }

    /// Splits `values` (a family-2 record's) for an IC of `kind`.
    pub fn decode(kind: u32, values: &[f64]) -> Result<Seed<'_>, String> {
        let short = || format!("carry: an IC record of kind {kind} is cut short");
        let at = |i: usize| values.get(i).copied().ok_or_else(short);
        let count = |v: f64, what: &str| -> Result<usize, String> {
            if v.is_finite() && v >= 0.0 && v.fract() == 0.0 && v <= values.len() as f64 {
                Ok(v as usize)
            } else {
                Err(format!("carry: an IC's {what} count {v} is out of range"))
            }
        };
        if at(0)? != kind as f64 {
            return Err(format!("carry: an IC record of kind {} on a kind {kind}", at(0)?));
        }
        let tag = at(1)?;
        if tag != tag_of(kind) {
            return Err(format!("carry: family tag {tag} on kind {kind}"));
        }
        let family_len = match tag {
            COMBINATIONAL => 1 + count(at(2)?, "Schmitt")?,
            SEQUENTIAL => match kind {
                crate::kinds::IC_74LS161 => 2,
                crate::kinds::IC_74LS173 => 5,
                _ => 6,
            },
            RAM_189 => 17,
            EEPROM => 12,
            _ => 1,
        };
        let family = values.get(2..2 + family_len).ok_or_else(short)?;
        let mut i = 2 + family_len;
        let reads = match at(i)? {
            -1.0 => {
                i += 1;
                None
            }
            n => {
                let n = count(n, "reads")?;
                let bits = values.get(i + 1..i + 1 + n).ok_or_else(short)?.iter().map(|&b| bit_of(b)).collect::<Result<Vec<_>, _>>()?;
                i += 1 + n;
                Some(bits)
            }
        };
        let n = count(at(i)?, "slot")?;
        let slots = values.get(i + 1..i + 1 + n).ok_or_else(short)?;
        i += 1 + n;
        let n = count(at(i)?, "driver")?;
        let triples = values.get(i + 1..i + 1 + 3 * n).ok_or_else(short)?;
        let drivers = triples
            .chunks_exact(3)
            .map(|t| {
                let committed = mode_of(t[0])?;
                let pending = match (mode_of(t[1])?, t[2]) {
                    (None, due) if due.is_nan() => None,
                    (Some(target), due) if due.is_finite() => Some((target, due)),
                    _ => return Err(format!("carry: a pending commit ({}, {}) is malformed", t[1], t[2])),
                };
                Ok((committed, pending))
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok(Seed { family, reads, slots, drivers })
    }
}

/// The slow layout: `[failed, stress, temperature, shutdown, ptc_tripped,
/// ptc_stress, ptc_rt, t_hold]`, at the engine's time. `NaN` for a quantity
/// the device has none of (no stress integral, no thermal model, not a PTC);
/// `failed` is 0 or 1. `t_hold` is the temperature an LED's junction law
/// holds (the thermal model's last boundary). Its latch records are their
/// own family. A seed reads `NaN` as "start fresh", which is how the host
/// drops a thermal model's state onto another part.
pub mod slow {
    pub const FAILED: usize = 0;
    pub const STRESS: usize = 1;
    pub const TEMPERATURE: usize = 2;
    pub const SHUTDOWN: usize = 3;
    pub const PTC_TRIPPED: usize = 4;
    pub const PTC_STRESS: usize = 5;
    pub const PTC_RT: usize = 6;
    pub const T_HOLD: usize = 7;
    pub const LEN: usize = 8;
}

/// The values a seed reads per family (a longer record's tail is a later
/// layout's, ignored).
fn min_len(family: u32) -> Option<usize> {
    Some(match family {
        FAMILY_CAPACITOR => 1,
        FAMILY_IC => 2,
        FAMILY_BATTERY => 6,
        FAMILY_PSU => 1,
        FAMILY_SLOW => slow::LEN,
        FAMILY_LATCH => 6,
        _ => return None,
    })
}

/// Whether a record is one this core seeds: a family it knows, at the
/// version it writes. Any other reads as none.
pub fn known(r: &Record) -> bool {
    r.version == VERSION && min_len(r.family).is_some()
}

/// Checks a program's `CARRY` block against its devices (program v4): the
/// records whole, each known one on a device of its family's kind, at least
/// its layout's length, with values in range, and at most one per device and
/// family (latches excepted: a device may have several). A malformed block
/// is a malformed program, so nothing is seeded from it.
pub fn validate(devices: &[crate::program::Device], block: &[f64]) -> Result<(), String> {
    use crate::kinds;
    let mut seen = std::collections::BTreeSet::new();
    for r in records(block)? {
        let d = devices.get(r.device).ok_or_else(|| format!("carry: device {} is out of range for {} device(s)", r.device, devices.len()))?;
        if !known(&r) {
            continue;
        }
        let what = format!("carry: device {}'s family {} record", r.device, r.family);
        let need = min_len(r.family).expect("known");
        if r.values.len() < need {
            return Err(format!("{what} has {} value(s), at least {need} needed", r.values.len()));
        }
        if r.family != FAMILY_LATCH && !seen.insert((r.device, r.family)) {
            return Err(format!("{what} is repeated"));
        }
        let v = r.values;
        let flag = |x: f64| x == 0.0 || x == 1.0;
        let finite_or_none = |x: f64| x.is_finite() || x.is_nan();
        let ok = match r.family {
            FAMILY_CAPACITOR => d.kind == kinds::CAPACITOR && v[0].is_finite(),
            FAMILY_IC => {
                if !crate::digital::is_ic(d.kind) {
                    return Err(format!("{what} is on device kind {}, no IC", d.kind));
                }
                ic::decode(d.kind, v).map_err(|e| format!("{what}: {e}"))?;
                true
            }
            FAMILY_BATTERY => d.kind == kinds::BATTERY_PACK && (0.0..=1.0).contains(&v[0]) && v[1..6].iter().all(|x| x.is_finite()),
            FAMILY_PSU => d.kind == kinds::BENCH_PSU && crate::devices::sources::Regime::from_code(v[0]).is_some(),
            FAMILY_SLOW => {
                use slow::*;
                flag(v[FAILED])
                    && [STRESS, TEMPERATURE, PTC_STRESS, PTC_RT, T_HOLD].iter().all(|&k| finite_or_none(v[k]))
                    && (flag(v[SHUTDOWN]) || v[SHUTDOWN].is_nan())
                    && (flag(v[PTC_TRIPPED]) || v[PTC_TRIPPED].is_nan())
            }
            FAMILY_LATCH => {
                crate::slow::failures::FailureKind::from_code(v[0]).is_some() && v[1] >= -1.0 && v[1].fract() == 0.0 && v[1] < u32::MAX as f64 && v[2].is_finite()
            }
            _ => unreachable!("known families only"),
        };
        if !ok {
            return Err(format!("{what} does not fit device kind {} or has a value out of range: {v:?}", d.kind));
        }
    }
    Ok(())
}

/// Ruling R57 D10 (a): the sum of the carried capacitor charges' `|vc|`,
/// what the cold-device span widens by. A charge held on a capacitor from a
/// free node to a fixed one is that node's potential's offset from the
/// fixed one: until it discharges, the free node can sit past every source
/// by as much, and the maximum principle bounds the rest by it. The sum is
/// over the whole program (a bound per section is no tighter than the one
/// span the bound keeps).
pub fn abs_vc_sum(block: &[f64]) -> f64 {
    records(block).map_or(0.0, |rs| rs.iter().filter(|r| known(r) && r.family == FAMILY_CAPACITOR).map(|r| r.values[0].abs()).sum())
}

/// A battery's carried SoC goes in as its record's `soc0`, the charge it
/// loads with, so every load-time value read from it (the rail's open-circuit
/// voltage, each partition's fixed node, the cell state) is the carried
/// charge's. `None` when nothing carries a battery.
pub fn with_carried_soc(program: &crate::program::Program) -> Option<crate::program::Program> {
    let rs = records(&program.carry).ok()?;
    let socs: Vec<(usize, f64)> = rs.iter().filter(|r| known(r) && r.family == FAMILY_BATTERY).map(|r| (r.device, r.values[0])).collect();
    if socs.is_empty() {
        return None;
    }
    let mut seeded = program.clone();
    for (d, soc) in socs {
        seeded.devices[d].params[crate::kinds::battery_param::SOC0] = soc;
    }
    Some(seeded)
}
