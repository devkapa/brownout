//! Program format v3 reader (core/README.md, "Program format, version 3").
//!
//! One `&[f64]`: header (magic, version, node count, device count, an
//! options block, then the readout slot counts), D device records, trailer.
//! Every field is an f64 carrying an exact integer, so reading a count or an
//! index means checking it is finite, non-negative, whole, and in range
//! *before* trusting it — the whole point of this module is that no
//! malformed input reaches a `Vec::with_capacity`, an index, or a match on
//! an unvalidated discriminant. A malformed program is `BoError` with
//! `ERR_MALFORMED` (or, for a well-formed-but-unrecognized kind code,
//! `ERR_UNKNOWN_KIND`) — never a panic.
//!
//! Per-kind shape and parameter rules live in `kinds.rs` (orchestrator-owned):
//! this module dispatches to `kinds::shape(code)` for pin/param/blob counts
//! and `check`, and never hard-codes a kind's own rules.

use crate::error::BoError;
use crate::kinds::{self, BlobShape};

pub const MAGIC: f64 = 0x424F4331 as f64; // "BOC1"
pub const VERSION: f64 = 3.0;

/// Pin record flag bits (core/README.md, "Device record"). Every other bit
/// must be 0 — since these are the only two defined bits, "no unknown bit
/// set" is exactly "flags <= PIN_FLAG_OPEN | PIN_FLAG_OPEN_HIGH".
pub const PIN_FLAG_OPEN: u32 = 1;
pub const PIN_FLAG_OPEN_HIGH: u32 = 2;
const PIN_FLAGS_MASK: u32 = PIN_FLAG_OPEN | PIN_FLAG_OPEN_HIGH;

/// A hash state is an integer in `[0, 2^32)` (core/README.md, "Device
/// record"). `2^32` itself, as an exact f64.
const HASH_STATE_BOUND: f64 = 4_294_967_296.0;

// Any field read as a "how many more f64s follow" count is bounded by the
// data actually remaining (see `Reader::count`), which already rules out
// runaway allocation from a corrupt length. `N` and `D` are not counts of
// upcoming fields in that sense, so they get an explicit, generous cap
// instead — high enough that no real circuit hits it, low enough that a
// garbage header can't ask this build to allocate gigabytes before any other
// check runs. The readout slot counts get the same treatment.
const MAX_NODES_OR_DEVICES: usize = 1_000_000;
const MAX_SLOT_COUNT: usize = 1_000_000;

/// A pin record: `node, flags, openHash, deadHash` (core/README.md, "Device
/// record"). Pin order is fixed per kind by the kind table (G7); this struct
/// carries one already-validated slot of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinRecord {
    pub node: usize,
    pub flags: u32,
    pub open_hash: u32,
    pub dead_hash: u32,
}

#[derive(Debug, Clone)]
pub struct Device {
    pub kind: u32,
    /// Index into the TS side's `circuit.components` — opaque to Rust,
    /// carried only so error messages and future tooling can point at it.
    pub component_index: usize,
    pub pins: Vec<PinRecord>,
    pub params: Vec<f64>,
    /// Per-kind meaning (core/README.md's record-layout table); read and
    /// kept, not interpreted here. Empty for every W0/W1a kind.
    pub blob: Vec<f64>,
}

/// The header's option block, resolved to typed fields with every key's
/// default already applied (core/README.md, "Options"). `key` numbers below
/// refer to that table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Options {
    /// Key 1: `0` backward Euler, `1` trapezoidal.
    pub integration: u8,
    /// Key 2: the solution-cache exactness switch.
    pub no_solution_cache: bool,
    /// Key 3: the rail-shortcut exactness switch.
    pub no_rail_shortcut: bool,
    /// Key 4: force the stepped path (tests only).
    pub force_stepped: bool,
    /// Key 5: ambient temperature, °C.
    pub ambient_c: f64,
    /// Key 6: `0` is the only value in W1 (classic's hash floating model).
    pub floating_model: u8,
    /// Key 7: the absolute time the loaded program starts at, seconds. Exists
    /// because classic keeps absolute `simTime` across a reload, and
    /// time-dependent behaviour keys on it (floating-input buckets, §1.5, and
    /// waveform source phase) — the facade passes the time it carries over,
    /// so the engine clock and the kernel both start here instead of at 0.
    pub start_time: f64,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            integration: 0,
            no_solution_cache: false,
            no_rail_shortcut: false,
            force_stepped: false,
            ambient_c: 25.0,
            floating_model: 0,
            start_time: 0.0,
        }
    }
}

/// Parses a `0`/`1` switch value into a `bool`. Shared by the load-time
/// option reader below (wrapped as `ERR_MALFORMED`) and `Engine::set_option`
/// (wrapped as `ERR_INVALID_ARGUMENT`) — the same value rule, two different
/// error codes depending on whether it is read at load or after.
pub(crate) fn switch_value(value: f64) -> Option<bool> {
    if value == 0.0 {
        Some(false)
    } else if value == 1.0 {
        Some(true)
    } else {
        None
    }
}

#[derive(Debug)]
pub struct Program {
    pub node_count: usize,
    pub options: Options,
    pub digital_slot_count: usize,
    pub channel_slot_count: usize,
    pub devices: Vec<Device>,
}

struct Reader<'a> {
    data: &'a [f64],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(data: &'a [f64]) -> Self {
        Reader { data, pos: 0 }
    }

    fn remaining(&self) -> usize {
        self.data.len() - self.pos
    }

    fn raw(&mut self, what: &str) -> Result<f64, BoError> {
        if self.pos >= self.data.len() {
            return Err(BoError::malformed(format!(
                "truncated program: expected {what} at offset {}, but the program ends at {}",
                self.pos,
                self.data.len()
            )));
        }
        let v = self.data[self.pos];
        self.pos += 1;
        Ok(v)
    }

    /// Reads one field that must be a magic/tag value, compared exactly.
    fn expect(&mut self, what: &str, expected: f64) -> Result<(), BoError> {
        let v = self.raw(what)?;
        if v != expected {
            return Err(BoError::malformed(format!(
                "{what}: expected {expected}, got {v}"
            )));
        }
        Ok(())
    }

    /// A non-negative whole number with no further bound — for a field that
    /// is not itself a count of upcoming data (there is nothing to bound it
    /// against but a fixed ceiling).
    fn bounded_index(&mut self, what: &str, max: usize) -> Result<usize, BoError> {
        let v = self.raw(what)?;
        Self::check_whole(what, v)?;
        if v > max as f64 {
            return Err(BoError::malformed(format!(
                "{what}: {v} exceeds the maximum of {max}"
            )));
        }
        Ok(v as usize)
    }

    /// A non-negative whole number that means "this many `fields_per_record`
    /// -sized records follow immediately". Bounding it by what is actually
    /// left in the buffer is both the correctness check (a legitimate count
    /// can never exceed the remaining data) and the allocation guard (a
    /// `Vec` sized from `n` below can never run away). `fields_per_record` is
    /// 1 for a flat list (params, blob) and larger for a fixed-width record
    /// (4 for a pin record, 2 for an option pair) — dividing first avoids
    /// overflow from multiplying an attacker-controlled count.
    fn count_of(&mut self, what: &str, fields_per_record: usize) -> Result<usize, BoError> {
        let v = self.raw(what)?;
        Self::check_whole(what, v)?;
        let n = v as usize;
        if n > self.remaining() / fields_per_record {
            return Err(BoError::malformed(format!(
                "{what}: {v} would need {} more f64s but only {} remain",
                n.saturating_mul(fields_per_record),
                self.remaining()
            )));
        }
        Ok(n)
    }

    /// A flat count of upcoming f64s (`fields_per_record` = 1).
    fn count(&mut self, what: &str) -> Result<usize, BoError> {
        self.count_of(what, 1)
    }

    fn check_whole(what: &str, v: f64) -> Result<(), BoError> {
        if !v.is_finite() {
            return Err(BoError::malformed(format!("{what}: {v} is not finite")));
        }
        if v < 0.0 {
            return Err(BoError::malformed(format!("{what}: {v} is negative")));
        }
        if v.fract() != 0.0 {
            return Err(BoError::malformed(format!("{what}: {v} is not a whole number")));
        }
        Ok(())
    }

    fn f64s(&mut self, what: &str, n: usize) -> Result<Vec<f64>, BoError> {
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            out.push(self.raw(&format!("{what}[{i}]"))?);
        }
        Ok(out)
    }

    /// Node indices are read the same way pin lists are, but validated
    /// against `node_count` rather than the remaining buffer.
    fn node_index(&mut self, what: &str, node_count: usize) -> Result<usize, BoError> {
        let v = self.raw(what)?;
        Self::check_whole(what, v)?;
        let idx = v as usize;
        if idx >= node_count {
            return Err(BoError::malformed(format!(
                "{what}: node index {idx} is out of range for {node_count} node(s)"
            )));
        }
        Ok(idx)
    }

    /// A hash state: integral and in `[0, 2^32)` (core/README.md, "Device
    /// record").
    fn hash_state(&mut self, what: &str) -> Result<u32, BoError> {
        let v = self.raw(what)?;
        Self::check_whole(what, v)?;
        if v >= HASH_STATE_BOUND {
            return Err(BoError::malformed(format!(
                "{what}: {v} is not in [0, 2^32)"
            )));
        }
        Ok(v as u32)
    }
}

impl Program {
    pub fn parse(data: &[f64]) -> Result<Program, BoError> {
        let mut r = Reader::new(data);

        r.expect("header magic", MAGIC)?;
        r.expect("version", VERSION)?;
        let node_count = r.bounded_index("node count N", MAX_NODES_OR_DEVICES)?;
        if node_count < 1 {
            return Err(BoError::malformed("node count N must be at least 1 (node 0 is the datum)"));
        }

        // D drives `Vec::with_capacity(device_count)` below, so it gets the
        // same "can't outrun the actual buffer" treatment as pinCount /
        // paramCount / blobCount. Every device record needs at least 5 more
        // f64s (kind, componentIndex, pinCount, paramCount, blobCount, even
        // at zero pins/params/blob). `remaining()` here, read before D
        // itself is consumed, still includes D's own field plus the option
        // count field, the S and C fields, and the 1-f64 trailer — five
        // fields that must exist even in the smallest legal program (O = 0,
        // no devices) — so that fixed tail is reserved before dividing by
        // the per-device minimum.
        const MIN_DEVICE_RECORD_LEN: usize = 5;
        const MIN_TAIL_AFTER_N: usize = 5; // D, option count O, S, C, trailer
        let max_devices_by_data = r.remaining().saturating_sub(MIN_TAIL_AFTER_N) / MIN_DEVICE_RECORD_LEN;
        let device_count = r.bounded_index("device count D", MAX_NODES_OR_DEVICES.min(max_devices_by_data))?;

        let option_count = r.count_of("option count O", 2)?;
        let mut options = Options::default();
        let mut seen_keys = [false; 8]; // indices 1..=7 used; 0 unused
        for i in 0..option_count {
            Self::parse_option(&mut r, i, &mut options, &mut seen_keys)?;
        }

        let digital_slot_count = r.bounded_index("digital slot count S", MAX_SLOT_COUNT)?;
        let channel_slot_count = r.bounded_index("channel slot count C", MAX_SLOT_COUNT)?;

        let mut devices = Vec::with_capacity(device_count);
        let mut claims = Vec::new();
        for d in 0..device_count {
            devices.push(Self::parse_device(&mut r, d, node_count, digital_slot_count, channel_slot_count, &mut claims)?);
        }
        Self::check_slot_ranges_disjoint(&mut claims)?;

        r.expect("trailer magic", MAGIC)?;
        if r.remaining() != 0 {
            return Err(BoError::malformed(format!(
                "{} f64(s) of trailing data after the trailer",
                r.remaining()
            )));
        }

        Ok(Program { node_count, options, digital_slot_count, channel_slot_count, devices })
    }

    /// Reads and applies one `(key, value)` option pair (core/README.md,
    /// "Options"). Every key at most once; each key's value range is the
    /// contract's table. Unknown key, repeated key or an out-of-range value
    /// is `ERR_MALFORMED` — this only ever runs at load, so a bad option in
    /// the program itself is a malformed program, not an invalid argument
    /// (that code, `-8`, is for `bo_engine_set_option` after load).
    fn parse_option(
        r: &mut Reader,
        index: usize,
        options: &mut Options,
        seen_keys: &mut [bool; 8],
    ) -> Result<(), BoError> {
        let key_f = r.raw(&format!("option[{index}] key"))?;
        Reader::check_whole("option key", key_f)?;
        let key = key_f as usize;
        if !(1..=7).contains(&key) {
            return Err(BoError::malformed(format!(
                "option[{index}]: unknown option key {key_f}"
            )));
        }
        if seen_keys[key] {
            return Err(BoError::malformed(format!(
                "option[{index}]: key {key} is repeated"
            )));
        }
        seen_keys[key] = true;

        let value = r.raw(&format!("option[{index}] value (key {key})"))?;
        match key {
            1 => {
                // INTEGRATION: 0 backward Euler, 1 trapezoidal.
                if value != 0.0 && value != 1.0 {
                    return Err(BoError::malformed(format!(
                        "option INTEGRATION: value must be 0 or 1, got {value}"
                    )));
                }
                options.integration = value as u8;
            }
            2 => {
                options.no_solution_cache = switch_value(value).ok_or_else(|| {
                    BoError::malformed(format!("option NO_SOLUTION_CACHE: value must be 0 or 1, got {value}"))
                })?;
            }
            3 => {
                options.no_rail_shortcut = switch_value(value).ok_or_else(|| {
                    BoError::malformed(format!("option NO_RAIL_SHORTCUT: value must be 0 or 1, got {value}"))
                })?;
            }
            4 => {
                options.force_stepped = switch_value(value).ok_or_else(|| {
                    BoError::malformed(format!("option FORCE_STEPPED: value must be 0 or 1, got {value}"))
                })?;
            }
            5 => {
                // AMBIENT_C: any finite temperature.
                if !value.is_finite() {
                    return Err(BoError::malformed(format!(
                        "option AMBIENT_C: {value} is not finite"
                    )));
                }
                options.ambient_c = value;
            }
            6 => {
                // FLOATING_MODEL: 0 (classic hash) is the only value in W1.
                if value != 0.0 {
                    return Err(BoError::malformed(format!(
                        "option FLOATING_MODEL: {value} is not a supported model (only 0 in W1)"
                    )));
                }
                options.floating_model = 0;
            }
            7 => {
                // START_TIME: finite seconds, >= 0 (core/README.md,
                // "Options"). Anything else at load is malformed, same as
                // every other option here; a bad value passed to
                // `bo_engine_set_option` after load is `-8` instead
                // (`Engine::set_option`, not this parser).
                if !value.is_finite() || value < 0.0 {
                    return Err(BoError::malformed(format!(
                        "option START_TIME: {value} must be finite and >= 0"
                    )));
                }
                options.start_time = value;
            }
            _ => unreachable!("key was already range-checked to 1..=7"),
        }
        Ok(())
    }

    fn parse_pin_record(
        r: &mut Reader,
        device_index: usize,
        pin_index: usize,
        node_count: usize,
    ) -> Result<PinRecord, BoError> {
        let node = r.node_index(
            &format!("device[{device_index}] pin[{pin_index}].node"),
            node_count,
        )?;
        let flags = r.bounded_index(
            &format!("device[{device_index}] pin[{pin_index}].flags"),
            PIN_FLAGS_MASK as usize,
        )? as u32;
        let open_hash = r.hash_state(&format!("device[{device_index}] pin[{pin_index}].openHash"))?;
        let dead_hash = r.hash_state(&format!("device[{device_index}] pin[{pin_index}].deadHash"))?;
        Ok(PinRecord { node, flags, open_hash, dead_hash })
    }

    /// Rejects two devices claiming the same digital slot, or the same channel
    /// slot (core/README.md, "Slot indices": the ranges are disjoint). The
    /// compiler never emits an overlap; the reader enforces it because the
    /// digital readout publishes only the slots whose value changed
    /// (`devices::ics::IcEvaluator::on_settled`), which is exact only while a
    /// slot has a single writer. Sorting the claims keeps this proportional to
    /// the device count, not to `S` or `C`.
    fn check_slot_ranges_disjoint(claims: &mut [SlotClaim]) -> Result<(), BoError> {
        claims.sort_by_key(|c| (c.channel, c.base, c.device));
        for pair in claims.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            if a.channel == b.channel && b.base < a.base + a.count {
                let what = if a.channel { "channel" } else { "digital" };
                return Err(BoError::malformed(format!(
                    "device[{}] ({}): {what} slots {}..{} overlap device[{}] ({})'s {}..{}",
                    b.device,
                    b.name,
                    b.base,
                    b.base + b.count,
                    a.device,
                    a.name,
                    a.base,
                    a.base + a.count
                )));
            }
        }
        Ok(())
    }

    fn parse_device(
        r: &mut Reader,
        index: usize,
        node_count: usize,
        digital_slot_count: usize,
        channel_slot_count: usize,
        claims: &mut Vec<SlotClaim>,
    ) -> Result<Device, BoError> {
        let kind_code = r.bounded_index(&format!("device[{index}] kind code"), u32::MAX as usize)? as u32;
        // componentIndex is opaque (an index into the TS side's
        // `circuit.components`, unrelated to how much program data remains),
        // so it is bounded by a fixed ceiling like the kind code, not by
        // `Reader::count`'s "must fit in what's left" rule.
        let component_index = r.bounded_index(&format!("device[{index}] component index"), u32::MAX as usize)?;

        let pin_count = r.count_of(&format!("device[{index}] pin count"), 4)?;
        let mut pins = Vec::with_capacity(pin_count);
        for p in 0..pin_count {
            pins.push(Self::parse_pin_record(r, index, p, node_count)?);
        }

        let param_count = r.count(&format!("device[{index}] param count"))?;
        let params = r.f64s(&format!("device[{index}] param"), param_count)?;
        // Every param must be finite (core/README.md: "every param and blob
        // value finite"). The TS compiler already refuses a non-finite
        // parameter before it ever reaches a program (falling back to the
        // classic engine instead), so this is a defense-in-depth check on
        // this side of the format boundary, not a case the physics needs to
        // handle.
        for (i, &value) in params.iter().enumerate() {
            if !value.is_finite() {
                return Err(BoError::malformed(format!(
                    "device[{index}] param[{i}]: {value} is not finite"
                )));
            }
        }

        let blob_count = r.count(&format!("device[{index}] blob count"))?;
        let blob = r.f64s(&format!("device[{index}] blob"), blob_count)?;
        for (i, &value) in blob.iter().enumerate() {
            if !value.is_finite() {
                return Err(BoError::malformed(format!(
                    "device[{index}] blob[{i}]: {value} is not finite"
                )));
            }
        }

        // Unknown-but-well-formed kind codes are ERR_UNKNOWN_KIND, distinct
        // from the structural ERR_MALFORMED above: the record parses fine,
        // it just names a kind this build has never heard of (core/README.md
        // ABI table, error -2). A kind code that fails `bounded_index`
        // (non-integer, negative, NaN, out of range) never reaches here —
        // that is a malformed field, not merely an unknown one.
        let shape = kinds::shape(kind_code).ok_or_else(|| {
            BoError::unknown_kind(format!("device[{index}]: unknown kind code {kind_code}"))
        })?;

        if pin_count < shape.pins || pin_count > shape.pins_max {
            let expected = if shape.pins == shape.pins_max {
                shape.pins.to_string()
            } else {
                format!("{}..={}", shape.pins, shape.pins_max)
            };
            return Err(BoError::malformed(format!(
                "device[{index}] ({}): expected {expected} pin(s), got {pin_count}",
                shape.name
            )));
        }
        if param_count != shape.params {
            return Err(BoError::malformed(format!(
                "device[{index}] ({}): expected {} param(s), got {param_count}",
                shape.name, shape.params
            )));
        }
        match shape.blob {
            BlobShape::Exact(n) => {
                if blob_count != n {
                    return Err(BoError::malformed(format!(
                        "device[{index}] ({}): expected exactly {n} blob value(s), got {blob_count}",
                        shape.name
                    )));
                }
            }
            BlobShape::AtLeast(min) => {
                if blob_count < min {
                    return Err(BoError::malformed(format!(
                        "device[{index}] ({}): expected at least {min} blob value(s), got {blob_count}",
                        shape.name
                    )));
                }
            }
            BlobShape::Table => {
                // `blob[0] = n` counts the (x, y) pairs after it. `n` is
                // compared as an f64 against what the blob can hold before it
                // is cast, so a huge `n` cannot overflow the length sum.
                let n = blob.first().copied().unwrap_or(0.0);
                let holds = blob_count.saturating_sub(1) / 2;
                if !(n >= 1.0 && n.fract() == 0.0 && n <= holds as f64 && blob_count == 1 + 2 * n as usize) {
                    return Err(BoError::malformed(format!(
                        "device[{index}] ({}): blob must be n, then n pairs, with n >= 1: got n = {n} and {blob_count} blob value(s)",
                        shape.name
                    )));
                }
            }
        }

        (shape.check)(pin_count, &params, &blob).map_err(|msg| {
            BoError::malformed(format!("device[{index}] ({}): {msg}", shape.name))
        })?;

        // The blob leads with the first slot of each readout kind the record
        // uses, digital before channel (core/README.md, "Slot indices");
        // every slot it owns must exist. The shapes size every such blob, so
        // a missing base means a shape table error, reported rather than
        // indexed past.
        let mut next_base = 0;
        for (what, total_name, count, total, channel) in [
            ("digital", "S", (shape.digital_slots)(pin_count), digital_slot_count, false),
            ("channel", "C", (shape.channel_slots)(pin_count), channel_slot_count, true),
        ] {
            if count == 0 {
                continue;
            }
            let base = *blob.get(next_base).ok_or_else(|| {
                BoError::malformed(format!("device[{index}] ({}): no blob value for its {what} slot base", shape.name))
            })?;
            next_base += 1;
            if base.fract() != 0.0 || base < 0.0 || base + count as f64 > total as f64 {
                return Err(BoError::malformed(format!(
                    "device[{index}] ({}): {what} slots {base}..{base}+{count} are outside {total_name} = {total}",
                    shape.name
                )));
            }
            claims.push(SlotClaim { channel, base: base as usize, count, device: index, name: shape.name });
        }

        Ok(Device { kind: kind_code, component_index, pins, params, blob })
    }
}

/// One device's claim on a run of readout slots, gathered while parsing so
/// [`Program::check_slot_ranges_disjoint`] can compare them.
struct SlotClaim {
    channel: bool,
    base: usize,
    count: usize,
    device: usize,
    name: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a well-formed program from the pieces validated tests want to
    /// break one at a time: header (with no options and zero readout
    /// slots), then `devices`, each already a flat `Vec<f64>` record, then
    /// the trailer. `build_full` is the general form; `build` is the common
    /// case of no options and no readout slots.
    fn build_full(node_count: f64, options: &[(f64, f64)], digital_slots: f64, channel_slots: f64, devices: &[Vec<f64>]) -> Vec<f64> {
        let mut out = vec![MAGIC, VERSION, node_count, devices.len() as f64, options.len() as f64];
        for &(k, v) in options {
            out.push(k);
            out.push(v);
        }
        out.push(digital_slots);
        out.push(channel_slots);
        for d in devices {
            out.extend_from_slice(d);
        }
        out.push(MAGIC);
        out
    }

    fn build(node_count: f64, devices: &[Vec<f64>]) -> Vec<f64> {
        build_full(node_count, &[], 0.0, 0.0, devices)
    }

    /// A pin record with no flags and zero hash states — what every test not
    /// specifically about flags or hashes wants.
    fn simple_pin(node: f64) -> Vec<f64> {
        vec![node, 0.0, 0.0, 0.0]
    }

    fn pin_with(node: f64, flags: f64, open_hash: f64, dead_hash: f64) -> Vec<f64> {
        vec![node, flags, open_hash, dead_hash]
    }

    fn resistor_record_with_pins(component_index: f64, a: Vec<f64>, b: Vec<f64>, r: f64) -> Vec<f64> {
        let mut out = vec![kinds::RESISTOR as f64, component_index, 2.0];
        out.extend(a);
        out.extend(b);
        out.push(1.0);
        out.push(r);
        out.push(0.0); // blobCount
        out
    }

    fn resistor_record(component_index: f64, a: f64, b: f64, r: f64) -> Vec<f64> {
        resistor_record_with_pins(component_index, simple_pin(a), simple_pin(b), r)
    }

    fn source_record(component_index: f64, pos: f64, neg: f64, v: f64) -> Vec<f64> {
        let mut out = vec![kinds::VOLTAGE_SOURCE as f64, component_index, 2.0];
        out.extend(simple_pin(pos));
        out.extend(simple_pin(neg));
        out.push(1.0);
        out.push(v);
        out.push(0.0); // blobCount
        out
    }

    // --- Basic structure, ported from v1 -----------------------------

    #[test]
    fn parses_a_well_formed_divider() {
        let data = build(3.0, &[
            source_record(0.0, 1.0, 0.0, 10.0),
            resistor_record(1.0, 1.0, 2.0, 1000.0),
            resistor_record(2.0, 2.0, 0.0, 1000.0),
        ]);
        let p = Program::parse(&data).expect("well-formed program parses");
        assert_eq!(p.node_count, 3);
        assert_eq!(p.devices.len(), 3);
        assert_eq!(p.devices[0].kind, kinds::VOLTAGE_SOURCE);
        assert_eq!(p.devices[0].pins[0].node, 1);
        assert_eq!(p.devices[0].pins[1].node, 0);
        assert_eq!(p.devices[0].params, vec![10.0]);
        assert_eq!(p.devices[1].kind, kinds::RESISTOR);
        assert_eq!(p.devices[1].component_index, 1);
        assert_eq!(p.options, Options::default());
        assert_eq!(p.digital_slot_count, 0);
        assert_eq!(p.channel_slot_count, 0);
    }

    #[test]
    fn empty_device_list_is_fine() {
        let data = build(1.0, &[]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.node_count, 1);
        assert!(p.devices.is_empty());
    }

    #[test]
    fn rejects_bad_header_magic() {
        let mut data = build(2.0, &[]);
        data[0] = 0.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_bad_version() {
        let mut data = build(2.0, &[]);
        data[1] = 1.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_zero_node_count() {
        let data = build(0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_negative_node_count() {
        let mut data = build(2.0, &[]);
        data[2] = -1.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_non_integer_node_count() {
        let mut data = build(2.0, &[]);
        data[2] = 2.5;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_nan_node_count() {
        let mut data = build(2.0, &[]);
        data[2] = f64::NAN;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_nan_device_count() {
        let mut data = build(2.0, &[]);
        data[3] = f64::NAN;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_truncated_header() {
        let data = vec![MAGIC, VERSION, 2.0];
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_truncated_device_record() {
        let mut data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, 1000.0)]);
        data.pop(); // drop the trailer
        data.pop(); // drop the last blob-count field too
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_node_index_out_of_range() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 2.0, 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_negative_node_index() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, -1.0, 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_non_integer_pin() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.5, 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_unknown_kind_code_as_dedicated_error() {
        let mut dev = vec![99.0, 0.0, 2.0];
        dev.extend(simple_pin(0.0));
        dev.extend(simple_pin(1.0));
        dev.push(1.0);
        dev.push(1000.0);
        dev.push(0.0);
        let data = build(2.0, &[dev]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -2, "unknown-but-well-formed kind code is ERR_UNKNOWN_KIND");
    }

    #[test]
    fn rejects_malformed_kind_code_as_malformed_not_unknown() {
        // A kind code that is not even a valid whole number is a structural
        // problem, not merely an unrecognized-but-legible one.
        let mut dev = vec![-1.0, 0.0, 2.0];
        dev.extend(simple_pin(0.0));
        dev.extend(simple_pin(1.0));
        dev.push(1.0);
        dev.push(1000.0);
        dev.push(0.0);
        let data = build(2.0, &[dev]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_wrong_pin_count_for_kind() {
        let mut dev = vec![kinds::RESISTOR as f64, 0.0, 3.0];
        dev.extend(simple_pin(0.0));
        dev.extend(simple_pin(1.0));
        dev.extend(simple_pin(2.0));
        dev.push(1.0);
        dev.push(1000.0);
        dev.push(0.0);
        let data = build(3.0, &[dev]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_wrong_param_count_for_kind() {
        let mut dev = vec![kinds::RESISTOR as f64, 0.0, 2.0];
        dev.extend(simple_pin(0.0));
        dev.extend(simple_pin(1.0));
        dev.push(2.0);
        dev.push(1000.0);
        dev.push(500.0);
        dev.push(0.0);
        let data = build(2.0, &[dev]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_wrong_blob_count_for_kind() {
        let mut rec = resistor_record(0.0, 0.0, 1.0, 1000.0);
        *rec.last_mut().unwrap() = 1.0; // blobCount = 1, but the shape says exactly 0
        rec.push(42.0);
        let data = build(2.0, &[rec]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_nan_param() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, f64::NAN)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_positive_infinity_param() {
        let data = build(2.0, &[source_record(0.0, 1.0, 0.0, f64::INFINITY)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_negative_infinity_param() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, f64::NEG_INFINITY)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_zero_resistor_r() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, 0.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_negative_resistor_r() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, -1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn accepts_a_tiny_positive_resistor_r() {
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, 1e-12)]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.devices[0].params, vec![1e-12]);
    }

    #[test]
    fn rejects_bad_trailer() {
        let mut data = build(2.0, &[]);
        *data.last_mut().unwrap() = 0.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_trailing_data() {
        let mut data = build(2.0, &[]);
        data.push(0.0);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_device_count_that_the_remaining_data_could_not_possibly_back() {
        let mut data = build(2.0, &[]);
        let d_index = 3;
        data[d_index] = 1_000_000.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_count_field_exceeding_remaining_data() {
        let data = build(2.0, &[vec![kinds::RESISTOR as f64, 0.0, 1000.0, 0.0, 1.0]]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn blob_round_trips_empty_for_both_w1a_kinds() {
        // Both current kinds are `BlobShape::Exact(0)` (core/README.md's
        // record-layout table), so a non-empty blob is a shape error
        // (`rejects_wrong_blob_count_for_kind` above), not something to read
        // and keep. The reader's `AtLeast` branch and its per-value
        // finiteness check are exercised once a kind with a non-empty blob
        // exists (W1c onward); nothing in W1a's kind set can reach it.
        let data = build(2.0, &[resistor_record(0.0, 0.0, 1.0, 1000.0)]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.devices[0].blob, Vec::<f64>::new());
    }

    // --- v2: options -----------------------------------------------------

    #[test]
    fn missing_options_take_their_defaults() {
        let data = build(2.0, &[]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.options, Options::default());
    }

    #[test]
    fn accepts_every_option_at_a_non_default_valid_value() {
        let data = build_full(
            2.0,
            &[(1.0, 1.0), (2.0, 1.0), (3.0, 1.0), (4.0, 1.0), (5.0, 30.0), (6.0, 0.0), (7.0, 2.5)],
            0.0,
            0.0,
            &[],
        );
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.options.integration, 1);
        assert!(p.options.no_solution_cache);
        assert!(p.options.no_rail_shortcut);
        assert!(p.options.force_stepped);
        assert_eq!(p.options.ambient_c, 30.0);
        assert_eq!(p.options.floating_model, 0);
        assert_eq!(p.options.start_time, 2.5);
    }

    #[test]
    fn start_time_defaults_to_zero_and_accepts_zero_explicitly() {
        let data = build_full(2.0, &[(7.0, 0.0)], 0.0, 0.0, &[]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.options.start_time, 0.0);
    }

    #[test]
    fn rejects_negative_or_non_finite_start_time() {
        for bad in [-1e-9, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let data = build_full(2.0, &[(7.0, bad)], 0.0, 0.0, &[]);
            let err = Program::parse(&data).unwrap_err();
            assert_eq!(err.code, -1, "START_TIME {bad} should be rejected");
        }
    }

    #[test]
    fn options_may_appear_in_any_order() {
        let data = build_full(2.0, &[(5.0, -10.0), (1.0, 1.0)], 0.0, 0.0, &[]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.options.integration, 1);
        assert_eq!(p.options.ambient_c, -10.0);
    }

    #[test]
    fn rejects_unknown_option_key() {
        for key in [0.0, 8.0, 100.0] {
            let data = build_full(2.0, &[(key, 0.0)], 0.0, 0.0, &[]);
            let err = Program::parse(&data).unwrap_err();
            assert_eq!(err.code, -1, "key {key} should be rejected");
        }
    }

    #[test]
    fn rejects_non_integer_option_key() {
        let data = build_full(2.0, &[(1.5, 0.0)], 0.0, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_duplicate_option_key() {
        let data = build_full(2.0, &[(2.0, 0.0), (2.0, 1.0)], 0.0, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_out_of_range_integration_value() {
        let data = build_full(2.0, &[(1.0, 2.0)], 0.0, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_out_of_range_exactness_switch_values() {
        for key in [2.0, 3.0, 4.0] {
            let data = build_full(2.0, &[(key, 0.5)], 0.0, 0.0, &[]);
            let err = Program::parse(&data).unwrap_err();
            assert_eq!(err.code, -1, "key {key} with value 0.5 should be rejected");
        }
    }

    #[test]
    fn rejects_non_finite_ambient_c() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let data = build_full(2.0, &[(5.0, bad)], 0.0, 0.0, &[]);
            let err = Program::parse(&data).unwrap_err();
            assert_eq!(err.code, -1);
        }
    }

    #[test]
    fn rejects_unsupported_floating_model() {
        let data = build_full(2.0, &[(6.0, 1.0)], 0.0, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    // --- v2: pin flags and hash states ------------------------------------

    #[test]
    fn rejects_unknown_pin_flag_bit() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 4.0, 0.0, 0.0), simple_pin(1.0), 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn accepts_both_pin_flag_bits_set() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 3.0, 0.0, 0.0), simple_pin(1.0), 1000.0)]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.devices[0].pins[0].flags, 3);
    }

    #[test]
    fn rejects_negative_pin_flags() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, -1.0, 0.0, 0.0), simple_pin(1.0), 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_negative_hash_state() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 0.0, -1.0, 0.0), simple_pin(1.0), 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_non_integer_hash_state() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 0.0, 1.5, 0.0), simple_pin(1.0), 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_hash_state_at_or_above_2_32() {
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 0.0, HASH_STATE_BOUND, 0.0), simple_pin(1.0), 1000.0)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn accepts_hash_state_at_2_32_minus_1() {
        let max = HASH_STATE_BOUND - 1.0;
        let data = build(2.0, &[resistor_record_with_pins(0.0, pin_with(0.0, 0.0, max, 0.0), simple_pin(1.0), 1000.0)]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.devices[0].pins[0].open_hash, 4_294_967_295);
    }

    // --- v2: readout slot counts ------------------------------------------

    #[test]
    fn digital_and_channel_slot_counts_round_trip() {
        let data = build_full(2.0, &[], 4.0, 2.0, &[]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.digital_slot_count, 4);
        assert_eq!(p.channel_slot_count, 2);
    }

    #[test]
    fn rejects_negative_digital_slot_count() {
        let data = build_full(2.0, &[], -1.0, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_non_integer_channel_slot_count() {
        let data = build_full(2.0, &[], 0.0, 1.5, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_nan_slot_counts() {
        let data = build_full(2.0, &[], f64::NAN, 0.0, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
        let data = build_full(2.0, &[], 0.0, f64::NAN, &[]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    // --- v2: truncation at every offset, NaN in every position ------------

    fn a_small_valid_program() -> Vec<f64> {
        build_full(3.0, &[(1.0, 1.0), (5.0, 30.0)], 1.0, 1.0, &[
            source_record(0.0, 1.0, 0.0, 10.0),
            resistor_record(1.0, 1.0, 2.0, 1000.0),
            resistor_record(2.0, 2.0, 0.0, 1000.0),
        ])
    }

    #[test]
    fn truncation_at_every_offset_is_rejected() {
        let good = a_small_valid_program();
        for i in 0..good.len() {
            let data = &good[..i];
            let err = Program::parse(data).unwrap_err();
            assert_eq!(err.code, -1, "truncating to {i} f64s did not fail as malformed");
        }
        // The untruncated buffer must still parse, or this test would be
        // vacuous (every prefix "failing" because the whole thing is bad).
        Program::parse(&good).expect("the untruncated program must be well-formed");
    }

    #[test]
    fn nan_in_every_position_is_rejected() {
        let good = a_small_valid_program();
        for i in 0..good.len() {
            let mut data = good.clone();
            data[i] = f64::NAN;
            let err = Program::parse(&data).unwrap_err();
            assert_eq!(err.code, -1, "position {i} set to NaN did not fail as malformed");
        }
    }

    // --- W1c: combinational IC records --------------------------------------

    /// A 74LS00 record: vcc, gnd, then 12 logic pins, 13 params, a blob of
    /// the slot base plus one flag per logic pin. `slot_base` and `flags`
    /// are what the tests below break.
    fn ls00_record(slot_base: f64, fam_flags: f64, pin_flag: f64) -> Vec<f64> {
        let mut rec = vec![kinds::IC_74LS00 as f64, 0.0, 14.0];
        rec.extend(simple_pin(1.0)); // vcc
        rec.extend(simple_pin(0.0)); // gnd
        for _ in 0..12 {
            rec.extend(simple_pin(2.0));
        }
        rec.push(kinds::ic_param::COUNT as f64);
        rec.extend([4.75, 5.25, fam_flags, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 20e-9]);
        rec.push(13.0);
        rec.push(slot_base);
        rec.extend([pin_flag; 12]);
        rec
    }

    #[test]
    fn accepts_an_ic_record_whose_slots_fit() {
        let p = Program::parse(&build_full(3.0, &[], 4.0, 0.0, &[ls00_record(0.0, 1.0, 1.0)])).unwrap();
        assert_eq!(p.devices[0].kind, kinds::IC_74LS00);
        assert_eq!(p.devices[0].blob.len(), 13);
    }

    #[test]
    fn rejects_an_ic_whose_slots_run_past_s() {
        let err = Program::parse(&build_full(3.0, &[], 4.0, 0.0, &[ls00_record(1.0, 1.0, 1.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("digital slots"), "{}", err.message);
    }

    #[test]
    fn rejects_a_fractional_ic_slot_base() {
        let err = Program::parse(&build_full(3.0, &[], 8.0, 0.0, &[ls00_record(0.5, 1.0, 1.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
    }

    #[test]
    fn rejects_an_undefined_ic_pin_flag_bit() {
        let err = Program::parse(&build_full(3.0, &[], 4.0, 0.0, &[ls00_record(0.0, 1.0, 4.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("pin flags"), "{}", err.message);
    }

    #[test]
    fn rejects_an_out_of_range_family_code() {
        let err = Program::parse(&build_full(3.0, &[], 4.0, 0.0, &[ls00_record(0.0, 4.0, 1.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("famFlags"), "{}", err.message);
    }

    // --- W1d retrofit: the 157/245 delay tail (VERSION 3) -------------------

    /// A 74LS245 record: vcc, gnd, then 18 logic pins, `param_count` params
    /// (15 IC params plus whatever tail the caller supplies, or a wrong
    /// count outright, to exercise the shape check), a blob of the slot base
    /// plus one flag per logic pin.
    fn ls245_record(param_count: usize) -> Vec<f64> {
        let mut rec = vec![kinds::IC_74LS245 as f64, 0.0, 20.0];
        rec.extend(simple_pin(1.0)); // vcc
        rec.extend(simple_pin(0.0)); // gnd
        for _ in 0..18 {
            rec.extend(simple_pin(2.0));
        }
        rec.push(param_count as f64);
        let mut params = vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 20e-9];
        params.resize(param_count, 0.0); // pads with 0.0 (a valid tail value) or truncates
        rec.extend(params);
        rec.push(19.0);
        rec.push(0.0);
        rec.extend([1.0; 18]);
        rec
    }

    #[test]
    fn rejects_a_74ls245_record_with_the_pre_w1d_13_param_count() {
        // Before this retrofit a 245 carried the 13 plain IC params and
        // nothing else; VERSION 3's shape needs 13 + 2*2 = 17 (the oeEnable
        // and oeDisable pairs), so the old record shape is now malformed.
        let data = build(3.0, &[ls245_record(kinds::ic_param::COUNT)]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("param"), "{}", err.message);
    }

    #[test]
    fn accepts_a_74ls245_record_with_its_delay_tail() {
        // 16 digital slots: the 245's own shape (every a/b pin is
        // output-capable, core/README.md's IC logic pins table).
        let p = Program::parse(&build_full(3.0, &[], 16.0, 0.0, &[ls245_record(kinds::ic_param::COUNT + 4)])).unwrap();
        assert_eq!(p.devices[0].kind, kinds::IC_74LS245);
        assert_eq!(p.devices[0].params.len(), 17);
    }

    // --- W1d: sequential and memory IC records ------------------------------

    /// A W1d IC record: vcc, gnd, `logic` pins on node 2, the 13 IC params
    /// (LS-TTL), a delay tail and `extra` params, then the blob.
    fn w1d_record(kind: u32, logic: usize, tail: &[f64], extra: &[f64], pin_flag: f64, words: &[f64]) -> Vec<f64> {
        let mut rec = vec![kind as f64, 0.0, (2 + logic) as f64];
        rec.extend(simple_pin(1.0));
        rec.extend(simple_pin(0.0));
        for _ in 0..logic {
            rec.extend(simple_pin(2.0));
        }
        let mut params = vec![4.75, 5.25, 1.0, 1.0, 0.8, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.008, 20e-9];
        params.extend_from_slice(tail);
        params.extend_from_slice(extra);
        rec.push(params.len() as f64);
        rec.extend(params);
        rec.push((1 + logic + words.len()) as f64);
        rec.push(0.0);
        rec.extend(std::iter::repeat(pin_flag).take(logic));
        rec.extend_from_slice(words);
        rec
    }

    const LS161_TAIL: [f64; 8] = [13e-9, 18e-9, 20e-9, 20e-9, 20e-9, 18e-9, 9e-9, 9e-9];

    #[test]
    fn accepts_a_74ls161_record_with_its_delay_tail() {
        let rec = w1d_record(kinds::IC_74LS161, 14, &LS161_TAIL, &[], 1.0, &[]);
        let p = Program::parse(&build_full(3.0, &[], 5.0, 0.0, &[rec])).unwrap();
        assert_eq!(p.devices[0].params.len(), 21);
    }

    #[test]
    fn rejects_a_negative_delay_tail_value() {
        let mut tail = LS161_TAIL;
        tail[3] = -1e-9;
        let err = Program::parse(&build_full(3.0, &[], 5.0, 0.0, &[w1d_record(kinds::IC_74LS161, 14, &tail, &[], 1.0, &[])])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("delay-tail"), "{}", err.message);
    }

    #[test]
    fn rejects_open_collector_on_a_register() {
        let err = Program::parse(&build_full(3.0, &[], 5.0, 0.0, &[w1d_record(kinds::IC_74LS161, 14, &LS161_TAIL, &[], 3.0, &[])])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("OPEN_COLLECTOR"), "{}", err.message);
    }

    fn eeprom_tail() -> [f64; 10] {
        [150e-9, 150e-9, 150e-9, 150e-9, 50e-9, 50e-9, 70e-9, 70e-9, 50e-9, 50e-9]
    }

    #[test]
    fn accepts_a_28c16_image_and_rejects_a_word_out_of_range() {
        let mut words = vec![4_294_967_295.0; 512];
        let ok = w1d_record(kinds::IC_28C16, 22, &eeprom_tail(), &[10e-3, 0.0], 1.0, &words);
        assert!(Program::parse(&build_full(3.0, &[], 8.0, 0.0, &[ok])).is_ok());
        words[7] = 4_294_967_296.0;
        let bad = w1d_record(kinds::IC_28C16, 22, &eeprom_tail(), &[10e-3, 0.0], 1.0, &words);
        let err = Program::parse(&build_full(3.0, &[], 8.0, 0.0, &[bad])).unwrap_err();
        assert!(err.message.contains("ROM words"), "{}", err.message);
    }

    #[test]
    fn rejects_sdp_on_a_28c16() {
        let rec = w1d_record(kinds::IC_28C16, 22, &eeprom_tail(), &[10e-3, 1.0], 1.0, &vec![0.0; 512]);
        let err = Program::parse(&build_full(3.0, &[], 8.0, 0.0, &[rec])).unwrap_err();
        assert!(err.message.contains("28c16"), "{}", err.message);
    }

    // --- W1e: LEDs, displays and the NE555 ------------------------------------

    /// A record of `kind` with `pins` pins on nodes 1, 0, 1, 0, ..., the given
    /// params and blob.
    fn record(kind: u32, pins: usize, params: &[f64], blob: &[f64]) -> Vec<f64> {
        let mut rec = vec![kind as f64, 0.0, pins as f64];
        for i in 0..pins {
            rec.extend(simple_pin(((i + 1) % 2) as f64));
        }
        rec.push(params.len() as f64);
        rec.extend_from_slice(params);
        rec.push(blob.len() as f64);
        rec.extend_from_slice(blob);
        rec
    }

    /// `[vf, n, iRated, isSet, is, burnt]`.
    fn led_record(n: f64, is_set: f64, is: f64, burnt: f64) -> Vec<f64> {
        record(kinds::LED, 2, &[1.8, n, 0.02, is_set, is, burnt], &[])
    }

    fn rgb_led_record(channel_base: f64) -> Vec<f64> {
        record(kinds::RGB_LED, 4, &[1.8, 2.0, 3.0, 0.02, 0.03, 0.0], &[channel_base])
    }

    /// The 13 IC params as the compiler resolves them for a catalog 555, the
    /// forced zeros included; `blob_flag` fills the six pin flags.
    fn ne555_record(params: [f64; 13], blob_flag: f64) -> Vec<f64> {
        let mut blob = vec![0.0];
        blob.extend([blob_flag; 6]);
        record(kinds::NE555, 8, &params, &blob)
    }

    const NE555_PARAMS: [f64; 13] = [4.5, 16.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0];

    #[test]
    fn accepts_a_record_of_each_w1e_kind() {
        let data = build_full(2.0, &[], 1.0, 3.0 + 2.0 + 8.0 + 8.0, &[
            led_record(2.0, 0.0, 0.0, 0.0),
            led_record(1.5, 1.0, 1e-18, 1.0),
            rgb_led_record(0.0),
            record(kinds::BICOLOR_LED, 3, &[1.8, 2.0, 0.02, 0.02, 1.0], &[3.0]),
            record(kinds::SEG7_CC, 10, &[2.0, 0.02], &[5.0]),
            record(kinds::SEG7_CA, 10, &[2.0, 0.02], &[13.0]),
            ne555_record(NE555_PARAMS, 0.0),
        ]);
        let p = Program::parse(&data).unwrap();
        let kinds_read: Vec<u32> = p.devices.iter().map(|d| d.kind).collect();
        assert_eq!(kinds_read, vec![kinds::LED, kinds::LED, kinds::RGB_LED, kinds::BICOLOR_LED, kinds::SEG7_CC, kinds::SEG7_CA, kinds::NE555]);
    }

    #[test]
    fn rejects_a_channel_base_out_of_range() {
        let ok = build_full(2.0, &[], 0.0, 4.0, &[rgb_led_record(1.0)]);
        assert!(Program::parse(&ok).is_ok(), "slots 1..4 fit C = 4");
        for base in [2.0, 0.5, -1.0] {
            let err = Program::parse(&build_full(2.0, &[], 0.0, 4.0, &[rgb_led_record(base)])).unwrap_err();
            assert_eq!(err.code, -1, "base {base}: {}", err.message);
            assert!(err.message.contains("channel slots"), "{}", err.message);
        }
    }

    #[test]
    fn rejects_two_devices_whose_digital_slot_ranges_overlap() {
        // A 74ls00 owns 4 digital slots. Bases 0 and 4 tile S = 8; base 2
        // reaches into the first one's 0..4. The readout publishes only
        // changed slots, which is exact only if a slot has one writer.
        let two = |second: f64| build_full(3.0, &[], 8.0, 0.0, &[ls00_record(0.0, 1.0, 1.0), ls00_record(second, 1.0, 1.0)]);
        assert!(Program::parse(&two(4.0)).is_ok(), "0..4 and 4..8 are disjoint");
        for second in [2.0, 0.0, 3.0] {
            let err = Program::parse(&two(second)).unwrap_err();
            assert_eq!(err.code, -1, "second base {second}: {}", err.message);
            assert!(err.message.contains("digital slots") && err.message.contains("overlap"), "{}", err.message);
            assert!(err.message.contains("device[1]") && err.message.contains("device[0]"), "names both devices: {}", err.message);
        }
        // Declaration order is not the slot order: the overlap is found either way round.
        let err = Program::parse(&build_full(3.0, &[], 8.0, 0.0, &[ls00_record(4.0, 1.0, 1.0), ls00_record(2.0, 1.0, 1.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
    }

    #[test]
    fn rejects_two_devices_whose_channel_slot_ranges_overlap() {
        // An rgb_led owns 3 channel slots.
        let two = |second: f64| build_full(2.0, &[], 0.0, 6.0, &[rgb_led_record(0.0), rgb_led_record(second)]);
        assert!(Program::parse(&two(3.0)).is_ok(), "0..3 and 3..6 are disjoint");
        for second in [2.0, 0.0, 1.0] {
            let err = Program::parse(&two(second)).unwrap_err();
            assert_eq!(err.code, -1, "second base {second}: {}", err.message);
            assert!(err.message.contains("channel slots") && err.message.contains("overlap"), "{}", err.message);
        }
    }

    #[test]
    fn a_digital_range_and_a_channel_range_may_share_index_numbers() {
        // Digital slot 0 and channel slot 0 are different arrays.
        let data = build_full(3.0, &[], 4.0, 3.0, &[ls00_record(0.0, 1.0, 1.0), rgb_led_record(0.0)]);
        assert!(Program::parse(&data).is_ok());
    }

    #[test]
    fn a_channel_base_is_checked_against_c_not_s() {
        let err = Program::parse(&build_full(2.0, &[], 3.0, 0.0, &[rgb_led_record(0.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("C = 0"), "{}", err.message);
    }

    #[test]
    fn rejects_a_led_with_n_zero_or_negative() {
        for n in [0.0, -2.0] {
            let err = Program::parse(&build(2.0, &[led_record(n, 0.0, 0.0, 0.0)])).unwrap_err();
            assert_eq!(err.code, -1, "n {n}: {}", err.message);
            assert!(err.message.contains("n must be > 0"), "{}", err.message);
        }
    }

    #[test]
    fn rejects_a_led_is_set_outside_0_1_and_a_stray_is() {
        let err = Program::parse(&build(2.0, &[led_record(2.0, 2.0, 0.0, 0.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("isSet"), "{}", err.message);
        let err = Program::parse(&build(2.0, &[led_record(2.0, 0.0, 1e-18, 0.0)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        assert!(err.message.contains("is must be 0"), "{}", err.message);
    }

    #[test]
    fn rejects_burnt_outside_0_1() {
        let err = Program::parse(&build(2.0, &[led_record(2.0, 0.0, 0.0, 0.5)])).unwrap_err();
        assert_eq!(err.code, -1, "{}", err.message);
        let bicolor = record(kinds::BICOLOR_LED, 3, &[1.8, 2.0, 0.02, 0.02, 2.0], &[0.0]);
        let err = Program::parse(&build_full(2.0, &[], 0.0, 2.0, &[bicolor])).unwrap_err();
        assert!(err.message.contains("burnt"), "{}", err.message);
    }

    #[test]
    fn rejects_an_ne555_with_a_forced_zero_set_or_a_pin_flag() {
        for (i, name) in [(7, "schmitt"), (8, "vtPlus"), (9, "vtMinus"), (11, "ioMax"), (12, "delay")] {
            let mut params = NE555_PARAMS;
            params[i] = if i == 7 { 1.0 } else { 0.2 };
            let err = Program::parse(&build_full(2.0, &[], 1.0, 0.0, &[ne555_record(params, 0.0)])).unwrap_err();
            assert_eq!(err.code, -1, "{name}: {}", err.message);
            assert!(err.message.contains(name), "{}", err.message);
        }
        let err = Program::parse(&build_full(2.0, &[], 1.0, 0.0, &[ne555_record(NE555_PARAMS, 1.0)])).unwrap_err();
        assert!(err.message.contains("pin flags must be 0"), "{}", err.message);
    }
}
