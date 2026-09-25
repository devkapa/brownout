//! The error codes are the contract (core/README.md, "C ABI, version 2").
//! Every fallible operation in this crate returns a `BoError` so the numeric
//! code and the human message are produced together, in one place, and can
//! never drift apart.

/// `-1`: magic/version/count/index mismatch, wrong param count, bad trailer,
/// trailing data — anything that means the bytes are not a well-formed
/// program at all.
pub const ERR_MALFORMED: i32 = -1;
/// `-2`: the bytes parse as a device record, but the kind code names no
/// kind this build knows about.
pub const ERR_UNKNOWN_KIND: i32 = -2;
/// `-3`: a well-formed program whose topology W0 cannot solve (a loop of
/// ideal voltage sources, or a floating one). The TS side falls back to the
/// classic engine on this code.
pub const ERR_UNSUPPORTED_TOPOLOGY: i32 = -3;
/// `-4`: a partition's dense solve hit a zero or non-finite pivot.
pub const ERR_SOLVE_FAILED: i32 = -4;
/// `-5`: the handle is unknown, was destroyed, or has no program loaded.
pub const ERR_BAD_HANDLE: i32 = -5;
/// `-6`: `bo_engine_advance` was asked to move time backwards.
pub const ERR_TIME_BACKWARDS: i32 = -6;
/// `-7`: the event kernel's per-instant delta-cycle pass cap was exceeded.
/// The host treats it as non-convergence, never a silent exit (core/README.md,
/// "Kernel semantics: events", point 3).
pub const ERR_DELTA_LIVELOCK: i32 = -7;
/// `-8`: invalid argument to a live ABI call — an unknown option key, a
/// value out of range, a load-time option changed after load, a device or
/// param index out of range, or a param that is not live.
pub const ERR_INVALID_ARGUMENT: i32 = -8;

#[derive(Debug, Clone)]
pub struct BoError {
    pub code: i32,
    pub message: String,
}

impl BoError {
    fn new(code: i32, message: impl Into<String>) -> Self {
        BoError { code, message: message.into() }
    }

    pub fn malformed(message: impl Into<String>) -> Self {
        Self::new(ERR_MALFORMED, message)
    }

    pub fn unknown_kind(message: impl Into<String>) -> Self {
        Self::new(ERR_UNKNOWN_KIND, message)
    }

    pub fn unsupported_topology(message: impl Into<String>) -> Self {
        Self::new(ERR_UNSUPPORTED_TOPOLOGY, message)
    }

    pub fn solve_failed(message: impl Into<String>) -> Self {
        Self::new(ERR_SOLVE_FAILED, message)
    }

    pub fn bad_handle(message: impl Into<String>) -> Self {
        Self::new(ERR_BAD_HANDLE, message)
    }

    pub fn time_backwards(message: impl Into<String>) -> Self {
        Self::new(ERR_TIME_BACKWARDS, message)
    }

    pub fn delta_livelock(message: impl Into<String>) -> Self {
        Self::new(ERR_DELTA_LIVELOCK, message)
    }

    pub fn invalid_argument(message: impl Into<String>) -> Self {
        Self::new(ERR_INVALID_ARGUMENT, message)
    }
}
