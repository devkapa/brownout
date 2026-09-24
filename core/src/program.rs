//! Program format v1 reader (core/README.md, "Program format, version 1").
//!
//! One `&[f64]`: header, D device records, trailer. Every field is an f64
//! carrying an exact integer, so reading a count or an index means checking
//! it is finite, non-negative, whole, and in range *before* trusting it —
//! the whole point of this module is that no malformed input reaches a
//! `Vec::with_capacity`, an index, or a match on an unvalidated discriminant.
//! A malformed program is `BoError` with `ERR_MALFORMED` (or, for a
//! well-formed-but-unrecognized kind code, `ERR_UNKNOWN_KIND`) — never a
//! panic.

use crate::error::BoError;

pub const MAGIC: f64 = 0x424F4331 as f64; // "BOC1"
pub const VERSION: f64 = 1.0;

// Any field read as a "how many more f64s follow" count is bounded by the
// data actually remaining (see `Reader::count`), which already rules out
// runaway allocation from a corrupt length. `N` and `D` are not counts of
// upcoming fields in that sense, so they get an explicit, generous cap
// instead — high enough that no real circuit hits it, low enough that a
// garbage header can't ask this build to allocate gigabytes before any other
// check runs.
const MAX_NODES_OR_DEVICES: usize = 1_000_000;

pub const KIND_RESISTOR: u32 = 1;
pub const KIND_VOLTAGE_SOURCE: u32 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceKind {
    Resistor,
    VoltageSource,
}

impl DeviceKind {
    fn from_code(code: u32) -> Option<Self> {
        match code {
            KIND_RESISTOR => Some(DeviceKind::Resistor),
            KIND_VOLTAGE_SOURCE => Some(DeviceKind::VoltageSource),
            _ => None,
        }
    }

    /// Pin and param counts are fixed per kind in v1 (core/README.md's kind
    /// table). Both current kinds happen to need 2 pins and 1 param; this
    /// stays a match (not a shared constant) so a v1 kind with a different
    /// shape is a one-line change, not a hunt for an assumption.
    fn pin_count(self) -> usize {
        match self {
            DeviceKind::Resistor => 2,
            DeviceKind::VoltageSource => 2,
        }
    }

    fn param_count(self) -> usize {
        match self {
            DeviceKind::Resistor => 1,
            DeviceKind::VoltageSource => 1,
        }
    }

    fn name(self) -> &'static str {
        match self {
            DeviceKind::Resistor => "resistor",
            DeviceKind::VoltageSource => "voltage_source",
        }
    }
}

#[derive(Debug, Clone)]
pub struct Device {
    pub kind: DeviceKind,
    /// Index into the TS side's `circuit.components` — opaque to Rust,
    /// carried only so error messages and future tooling can point at it.
    pub component_index: usize,
    pub pins: Vec<usize>,
    pub params: Vec<f64>,
    /// Empty in W0 (core/README.md: "The blob is empty in W0; from W1 it
    /// carries ROM/RAM contents and IC pin maps"). Read and kept, not
    /// interpreted — W1's kinds will give it meaning, W0's two kinds do not.
    pub blob: Vec<f64>,
}

#[derive(Debug)]
pub struct Program {
    pub node_count: usize,
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

    /// A non-negative whole number that means "this many more f64s follow
    /// immediately". Bounding it by what is actually left in the buffer is
    /// both the correctness check (a legitimate count can never exceed the
    /// remaining data) and the allocation guard (a `Vec` sized from `n`
    /// below can never run away).
    fn count(&mut self, what: &str) -> Result<usize, BoError> {
        let v = self.raw(what)?;
        Self::check_whole(what, v)?;
        if v as usize > self.remaining() {
            return Err(BoError::malformed(format!(
                "{what}: {v} exceeds the {} f64s remaining in the program",
                self.remaining()
            )));
        }
        Ok(v as usize)
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
        // D drives `Vec::with_capacity(device_count)` below, so — unlike N,
        // which is just a scalar with no data of its own following it — it
        // gets the same "can't outrun the actual buffer" treatment as
        // pinCount/paramCount/blobCount. Every device record needs at least
        // 5 more f64s (kind, componentIndex, pinCount, paramCount, blobCount,
        // even at zero pins/params/blob), so the data remaining after D
        // itself bounds how large D can honestly be. Without this, a 5-f64
        // buffer claiming D = 1,000,000 would reserve that capacity before
        // the first device's truncation error ever fires.
        const MIN_DEVICE_RECORD_LEN: usize = 5;
        // `remaining()` here still includes the D field itself (not yet
        // read) and must also leave room for the 1-f64 trailer after every
        // device record — hence `- 2`, not `- 1`.
        let max_devices_by_data = r.remaining().saturating_sub(2) / MIN_DEVICE_RECORD_LEN;
        let device_count = r.bounded_index("device count D", MAX_NODES_OR_DEVICES.min(max_devices_by_data))?;

        let mut devices = Vec::with_capacity(device_count);
        for d in 0..device_count {
            devices.push(Self::parse_device(&mut r, d, node_count)?);
        }

        r.expect("trailer magic", MAGIC)?;
        if r.remaining() != 0 {
            return Err(BoError::malformed(format!(
                "{} f64(s) of trailing data after the trailer",
                r.remaining()
            )));
        }

        Ok(Program { node_count, devices })
    }

    fn parse_device(r: &mut Reader, index: usize, node_count: usize) -> Result<Device, BoError> {
        let kind_code = r.bounded_index(&format!("device[{index}] kind code"), u32::MAX as usize)? as u32;
        // componentIndex is opaque (an index into the TS side's
        // `circuit.components`, unrelated to how much program data remains),
        // so it is bounded by a fixed ceiling like the kind code, not by
        // `Reader::count`'s "must fit in what's left" rule.
        let component_index = r.bounded_index(&format!("device[{index}] component index"), u32::MAX as usize)?;

        let pin_count = r.count(&format!("device[{index}] pin count"))?;
        let mut pins = Vec::with_capacity(pin_count);
        for p in 0..pin_count {
            pins.push(r.node_index(&format!("device[{index}] pin[{p}]"), node_count)?);
        }

        let param_count = r.count(&format!("device[{index}] param count"))?;
        let params = r.f64s(&format!("device[{index}] param"), param_count)?;
        // Every param must be finite (core/README.md: "every param finite").
        // The TS compiler already refuses a non-finite parameter before it
        // ever reaches a program (falling back to the classic engine
        // instead), so this is a defense-in-depth check on this side of the
        // format boundary, not a case the physics needs to handle.
        for (i, &value) in params.iter().enumerate() {
            if !value.is_finite() {
                return Err(BoError::malformed(format!(
                    "device[{index}] param[{i}]: {value} is not finite"
                )));
            }
        }

        let blob_count = r.count(&format!("device[{index}] blob count"))?;
        let blob = r.f64s(&format!("device[{index}] blob"), blob_count)?;

        // Unknown-but-well-formed kind codes are ERR_UNKNOWN_KIND, distinct
        // from the structural ERR_MALFORMED above: the record parses fine,
        // it just names a kind this build has never heard of (core/README.md
        // ABI table, error -2). A kind code that fails `bounded_index`
        // (non-integer, negative, NaN, out of range) never reaches here —
        // that is a malformed field, not merely an unknown one.
        let kind = DeviceKind::from_code(kind_code).ok_or_else(|| {
            BoError::unknown_kind(format!("device[{index}]: unknown kind code {kind_code}"))
        })?;

        if pin_count != kind.pin_count() {
            return Err(BoError::malformed(format!(
                "device[{index}] ({}): expected {} pin(s), got {pin_count}",
                kind.name(),
                kind.pin_count()
            )));
        }
        if param_count != kind.param_count() {
            return Err(BoError::malformed(format!(
                "device[{index}] ({}): expected {} param(s), got {param_count}",
                kind.name(),
                kind.param_count()
            )));
        }

        // A resistor's R is a divisor throughout the kernel (conductance
        // 1/R when stamping, and again in Ohm's law for its current); zero
        // or negative is nonsensical and, for zero, an immediate non-finite
        // result. The TS compiler already clamps `R >= 1e-3` (core/README.md
        // kind table), so a non-positive R here can only come from a
        // hand-built program, not a compiled circuit — this is a reader rule
        // for that case, same class of check as "every param finite".
        if kind == DeviceKind::Resistor && !(params[0] > 0.0) {
            return Err(BoError::malformed(format!(
                "device[{index}] (resistor): R = {} must be greater than 0",
                params[0]
            )));
        }

        Ok(Device { kind, component_index, pins, params, blob })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a well-formed program from the pieces validated tests want to
    /// break one at a time: header, then `devices`, each already a flat
    /// `Vec<f64>` record (kind, componentIndex, pinCount, pins..., paramCount,
    /// params..., blobCount, blobs...), then the trailer.
    fn build(node_count: f64, devices: &[Vec<f64>]) -> Vec<f64> {
        let mut out = vec![MAGIC, VERSION, node_count, devices.len() as f64];
        for d in devices {
            out.extend_from_slice(d);
        }
        out.push(MAGIC);
        out
    }

    fn resistor_record(component_index: f64, a: f64, b: f64, r: f64) -> Vec<f64> {
        vec![KIND_RESISTOR as f64, component_index, 2.0, a, b, 1.0, r, 0.0]
    }

    fn source_record(component_index: f64, pos: f64, neg: f64, v: f64) -> Vec<f64> {
        vec![KIND_VOLTAGE_SOURCE as f64, component_index, 2.0, pos, neg, 1.0, v, 0.0]
    }

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
        assert_eq!(p.devices[0].kind, DeviceKind::VoltageSource);
        assert_eq!(p.devices[0].pins, vec![1, 0]);
        assert_eq!(p.devices[0].params, vec![10.0]);
        assert_eq!(p.devices[1].kind, DeviceKind::Resistor);
        assert_eq!(p.devices[1].component_index, 1);
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
        data[1] = 2.0;
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
        let data = build(2.0, &[vec![99.0, 0.0, 2.0, 0.0, 1.0, 1.0, 1000.0, 0.0]]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -2, "unknown-but-well-formed kind code is ERR_UNKNOWN_KIND");
    }

    #[test]
    fn rejects_malformed_kind_code_as_malformed_not_unknown() {
        // A kind code that is not even a valid whole number is a structural
        // problem, not merely an unrecognized-but-legible one.
        let data = build(2.0, &[vec![-1.0, 0.0, 2.0, 0.0, 1.0, 1.0, 1000.0, 0.0]]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_wrong_pin_count_for_kind() {
        let data = build(3.0, &[vec![KIND_RESISTOR as f64, 0.0, 3.0, 0.0, 1.0, 2.0, 1.0, 1000.0, 0.0]]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_wrong_param_count_for_kind() {
        let data = build(2.0, &[vec![KIND_RESISTOR as f64, 0.0, 2.0, 0.0, 1.0, 2.0, 1000.0, 500.0, 0.0]]);
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
        // The compiler floors R at 1e-3 (core/README.md kind table), so this
        // can only reach the reader from a hand-built program — a defense in
        // depth check, same as the finiteness check above it.
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
        // Not <= 0, so this must still parse — only non-positive R is a
        // reader rule, not a lower magnitude bound (that is the compiler's
        // 1e-3 floor, which the reader does not itself enforce).
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
        // D claims a million devices; the buffer has room for none. This
        // must be rejected before any `Vec::with_capacity(device_count)`
        // runs, not merely once the first device record turns out truncated.
        let mut data = build(2.0, &[]);
        let d_index = 3;
        data[d_index] = 1_000_000.0;
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn rejects_count_field_exceeding_remaining_data() {
        // pinCount claims 1000 pins but the buffer has nothing like that left.
        let data = build(2.0, &[vec![KIND_RESISTOR as f64, 0.0, 1000.0, 0.0, 1.0]]);
        let err = Program::parse(&data).unwrap_err();
        assert_eq!(err.code, -1);
    }

    #[test]
    fn blob_is_read_but_not_required_to_be_empty() {
        // W0 kinds don't use the blob, but the reader must still be able to
        // walk past a non-empty one without misparsing the rest.
        let mut rec = resistor_record(0.0, 0.0, 1.0, 1000.0);
        // rewrite blobCount=2 and append two blob values
        let blob_count_idx = rec.len() - 1;
        rec[blob_count_idx] = 2.0;
        rec.push(1.0);
        rec.push(2.0);
        let data = build(2.0, &[rec]);
        let p = Program::parse(&data).unwrap();
        assert_eq!(p.devices[0].blob, vec![1.0, 2.0]);
    }
}
