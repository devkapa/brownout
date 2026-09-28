//! The floating/indeterminate model and its floating-pull stamp
//! (core/README.md, "Kernel semantics: the digital core (W1c)", points 5-6;
//! spec §1.5-1.6, `_floatingLogicHigh`/`_floatingNoise` sim-engine.ts:5029-5058,
//! `hash32` :1390-1397, `_stampFloatingDigitalInputs` :5182-5204). Owned by
//! W1c worker C1.
//!
//! Classic hashes the *whole* string `${compId}:${pinId}:${bucket}` (or the
//! `:undefined` dead-band variant) with FNV-1a on every read. The program
//! format instead ships the FNV-1a state already advanced over the id/pin
//! prefix as `openHash`/`deadHash` (core/README.md, "Device record") — the
//! component and pin id strings never reach Rust — so this module only ever
//! finishes that state over the bucket's decimal digits ([`finish`]) and
//! never sees a component or pin id.

use crate::digital::power::IcPower;
use crate::digital::{IcPin, IcRecord};

/// FNV-1a's `h = h * 0x01000193` step, continued over one more byte.
fn fnv1a_step(h: u32, byte: u8) -> u32 {
    (h ^ byte as u32).wrapping_mul(0x0100_0193)
}

/// Classic's 10 ms bucket (`sim-engine.ts:5030`): `floor((t + 1e-12) /
/// 0.01)`, computed in f64 exactly as classic computes it (the `1e-12`
/// nudge is there so a `t` that lands exactly on a boundary, after the
/// float error a chain of additions accumulates, still rounds into the
/// bucket it belongs to rather than the one before it). `t` is always a
/// simulation time, hence `>= 0`, so the floor is representable as `u64`
/// for any `t` this engine will ever see.
pub fn bucket(t: f64) -> u64 {
    (((t + 1e-12) / 0.01).floor()) as u64
}

/// The smallest `f64` strictly greater than `t` whose [`bucket`] is greater
/// than `bucket(t)` — the next instant a floating or dead-band read of a
/// pin keyed on `t`'s bucket can change (contract point 5: "every device
/// that reads such a pin gets an event at each bucket boundary").
///
/// The closed-form boundary — the real number `target * 0.01 - 1e-12`, where
/// `target = bucket(t) + 1` — is only an estimate: converting it to `f64`
/// rounds, and `bucket()`'s own division and floor can round again, so nothing
/// guarantees the literal result of that formula actually reads back as
/// `target`, or that it is the *smallest* `f64` that does. This walks from the
/// estimate by single ulps (`f64::next_up`/`next_down`) until both hold.
pub fn next_bucket_boundary(t: f64) -> f64 {
    let base = bucket(t);
    // Finding 4: at a large enough `t` (roughly `START_TIME >= 1.8447e17`
    // seconds, `u64::MAX * 0.01`), `bucket`'s own `as u64` cast saturates to
    // `u64::MAX` -- and stays saturated for every float near `t`, since a
    // single f64 ulp there already spans far more than one 10ms bucket.
    // `base + 1` would then overflow (wrapping to 0 in a release build,
    // panicking in debug/`cargo test`), and the climbing loop below (`while
    // bucket(candidate) <= base`) could never terminate: every candidate it
    // tries also saturates to the same `u64::MAX`, so the comparison holds
    // forever and `bo_engine_load` hangs instead of returning. No real
    // program reaches this (about 5.8 billion years of simulated time); "no
    // bucket boundary ever" is the only sane answer once the bucket index
    // itself can no longer be represented.
    if base == u64::MAX {
        return f64::INFINITY;
    }
    let target = base + 1;

    let mut candidate = (target as f64) * 0.01 - 1e-12;
    if candidate <= t {
        // The rounded estimate landed at or before t (can happen near t=0,
        // where 1e-12 is comparable to the boundary itself); start the
        // upward walk from t's own next representable value instead.
        candidate = t.next_up();
    }

    // Climb until the candidate is actually past the boundary -- defensively
    // guarded against ever spinning on a non-finite candidate: nothing in
    // this function's own arithmetic should reach one once the saturated-
    // `base` case above is excluded, but a hang here would be exactly
    // finding 4's failure mode, so the walk itself never assumes that rather
    // than checks it.
    while bucket(candidate) <= base {
        if !candidate.is_finite() {
            return f64::INFINITY;
        }
        candidate = candidate.next_up();
    }
    // ...then walk back down to the smallest float that still is.
    loop {
        let prev = candidate.next_down();
        if prev <= t || bucket(prev) <= base {
            break;
        }
        candidate = prev;
    }
    candidate
}

/// Classic's `hash32` state, continued over `bucket`'s decimal digits
/// (ASCII, most significant first — classic's `${...}:${bucket}` via JS
/// `Number#toString`, which never emits a sign or a leading zero for a
/// non-negative integer, matching `bucket`'s `u64`). `prefix` is the caller's
/// `openHash` or `deadHash`, already advanced over `${id}:${pin}:` or
/// `${id}:${pin}:undefined:`.
///
/// Digits are produced least-significant-first into a stack buffer and
/// hashed in reverse, rather than allocating a decimal string, because this
/// runs on the hot path of every open or dead-band pin read.
pub fn finish(prefix: u32, bucket: u64) -> u32 {
    let mut digits = [0u8; 20]; // u64::MAX has 20 decimal digits.
    let mut n = bucket;
    let mut i = digits.len();
    loop {
        i -= 1;
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    digits[i..].iter().fold(prefix, |h, &byte| fnv1a_step(h, byte))
}

/// `_floatingLogicHigh` (spec §1.5, sim-engine.ts:5029-5035): the coin flip
/// for a floating or dead-band read at time `t`. `prefix` is the pin's
/// `openHash` (a floating read) or `deadHash` (a dead-band read).
pub fn floating_high(prefix: u32, t: f64, ttl: bool) -> bool {
    let n = finish(prefix, bucket(t)) as f64 / 0xffff_ffffu32 as f64;
    let p_high = if ttl { 0.7 } else { 0.5 };
    n < p_high
}

/// `_stampFloatingDigitalInputs`, one pin (spec §1.6, sim-engine.ts:5182-5204):
/// while powered, an `OPEN` `FLOATING_INPUT` pin that is not `OPEN_HIGH`
/// (`IC_OPEN_HIGH_PINS` gets no pull at all, `:5196`) gets a Norton pull of
/// `g = 1/(TTL ? 1e6 : 1e7)` toward `power.gnd + (high ? power.vSupply : 0)`,
/// `high` from the plain open-key hash — the same key the read side uses for
/// an open pin, so the pin's voltage and its logic read agree. Returns
/// `(conductance, absolute target volts)`, or `None` when no pull applies.
pub fn pull(ic: &IcRecord, power: &IcPower, pin: &IcPin, t: f64) -> Option<(f64, f64)> {
    if !power.powered || !pin.open || !pin.floating_input || pin.open_high {
        return None;
    }
    let r = if ic.ttl { 1e6 } else { 1e7 };
    let high = floating_high(pin.open_hash, t, ic.ttl);
    let target = power.v_gnd + if high { power.vs } else { 0.0 };
    Some((1.0 / r, target))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A from-scratch FNV-1a, used only to build test prefixes the way the
    /// compiler builds `openHash`/`deadHash` — hashing the id/pin string the
    /// program format never sends to Rust — so the golden vectors are
    /// checked end to end through [`finish`], not assumed.
    fn fnv1a(s: &str) -> u32 {
        s.bytes().fold(0x811c_9dc5u32, fnv1a_step)
    }

    #[test]
    fn fnv1a_of_empty_string_is_the_basis() {
        // hash32("") = 0x811c9dc5 (spec §1.5 golden vector).
        assert_eq!(fnv1a(""), 0x811c_9dc5);
    }

    #[test]
    fn golden_vectors_u1_1b_open_key() {
        let prefix = fnv1a("u1:1b:");
        assert_eq!(finish(prefix, 0), 0x1d77_35be, "hash32(\"u1:1b:0\")");
        assert_eq!(finish(prefix, 1), 0x1e77_3751, "hash32(\"u1:1b:1\")");
    }

    #[test]
    fn golden_vector_u1_1b_dead_key() {
        let prefix = fnv1a("u1:1b:undefined:");
        assert_eq!(finish(prefix, 0), 0xb06a_5224, "hash32(\"u1:1b:undefined:0\")");
    }

    #[test]
    fn verdicts_on_the_golden_vectors() {
        // n = 0x1d7735be / 0xffffffff = 0.11510..., HIGH for both families
        // (pHigh 0.7 TTL, 0.5 CMOS) since 0.1151 < 0.5 < 0.7.
        let open_prefix = fnv1a("u1:1b:");
        let n0 = finish(open_prefix, 0) as f64 / 0xffff_ffffu32 as f64;
        assert!((n0 - 0.11510).abs() < 1e-4);
        assert!(floating_high(open_prefix, 0.0, true), "TTL HIGH at bucket 0");
        assert!(floating_high(open_prefix, 0.0, false), "CMOS HIGH at bucket 0");

        // The dead-band vector: n = 0xb06a5224 / 0xffffffff = 0.68912...,
        // which is above CMOS's 0.5 (LOW) but below TTL's 0.7 (HIGH).
        let dead_prefix = fnv1a("u1:1b:undefined:");
        let n_dead = finish(dead_prefix, 0) as f64 / 0xffff_ffffu32 as f64;
        assert!((n_dead - 0.68912).abs() < 1e-4);
        assert!(floating_high(dead_prefix, 0.0, true), "TTL HIGH in the dead band");
        assert!(!floating_high(dead_prefix, 0.0, false), "CMOS LOW in the dead band");
    }

    #[test]
    fn bucket_boundaries() {
        assert_eq!(bucket(0.0), 0);
        assert_eq!(bucket(0.00999999999), 0);
        assert_eq!(bucket(0.01), 1);
        assert_eq!(bucket(0.0199), 1);
        assert_eq!(bucket(0.02), 2);
    }

    #[test]
    fn next_bucket_boundary_properties() {
        for &t in &[0.0, 0.00999999999, 0.01, 0.019999999999, 1.0, 123.456, 9_999.999, 1.0e6, 3.14159e8] {
            let boundary = next_bucket_boundary(t);
            assert!(boundary > t, "boundary must be strictly after t={t}");
            assert_eq!(bucket(boundary), bucket(t) + 1, "boundary's bucket is exactly one past t's, t={t}");
            let just_below = boundary.next_down();
            // just_below is either <= t (t itself is the immediate predecessor
            // in float space) or still shares t's bucket; either way it must
            // NOT already be in the next bucket.
            assert!(just_below <= t || bucket(just_below) == bucket(t), "the float just below the boundary is not yet in the next bucket, t={t}");
        }
    }

    #[test]
    fn next_bucket_boundary_returns_infinity_once_bucket_saturates() {
        // Finding 4 [low]: at START_TIME = 1e18 (valid per the contract:
        // "finite seconds >= 0"), the OLD implementation hung forever inside
        // `bo_engine_load`'s settle -- `bucket(1e18)` saturates to
        // `u64::MAX`, `base + 1` overflows, and the climbing loop's
        // `bucket(candidate) <= base` never goes false because every
        // candidate near 1e18 saturates to that same `u64::MAX` too. This is
        // the exact scenario the finding names: "a program with START_TIME
        // = 1e18 ... and a powered IC with an open FLOATING_INPUT pin ...
        // bo_engine_load never returns."
        assert_eq!(bucket(1e18), u64::MAX, "1e18 must actually be past bucket()'s own saturation point for this test to mean anything");
        assert_eq!(next_bucket_boundary(1e18), f64::INFINITY, "no bucket boundary can be represented once the bucket index itself saturates");

        // A `t` comfortably below the saturation point (the finding's own
        // "threshold is START_TIME >= about 1.8447e17 s") must still behave
        // exactly as before: a real, finite next boundary, strictly ahead of
        // `t` and in a later bucket. Not `bucket(t) + 1` exactly: at this
        // magnitude a single f64 ulp already spans many buckets (unlike
        // `next_bucket_boundary_at_large_t`'s own 1e9, small enough that the
        // exact-successor guarantee still holds), so the walk can only land
        // on the smallest REPRESENTABLE float past the boundary, which is
        // free to be several buckets beyond `bucket(t) + 1` -- a pre-existing,
        // correct property of the walk at this magnitude, not something this
        // fix changes.
        let below_threshold = 1.0e17;
        assert!(bucket(below_threshold) < u64::MAX);
        let boundary = next_bucket_boundary(below_threshold);
        assert!(boundary.is_finite());
        assert!(boundary > below_threshold);
        assert!(bucket(boundary) > bucket(below_threshold));
    }

    #[test]
    fn next_bucket_boundary_at_large_t() {
        // A t large enough that many ulps separate consecutive buckets, but
        // not so large that a single ulp spans more than one bucket.
        let t = 1.0e9;
        let boundary = next_bucket_boundary(t);
        assert!(boundary > t);
        assert_eq!(bucket(boundary), bucket(t) + 1);
        assert_eq!(bucket(boundary.next_down()), bucket(t));
    }

    fn ic(ttl: bool) -> IcRecord {
        IcRecord {
            kind: 50,
            vcc: pin_at(1, false, false),
            gnd: pin_at(0, false, false),
            logic: Vec::new(),
            vcc_min: 4.75,
            vcc_max: 5.25,
            ttl,
            cmos_ratio: false,
            vil: Some(0.8),
            vih: Some(2.0),
            schmitt: None,
            nominal: 5.0,
            io_max: Some(0.008),
            delay: 20e-9,
            slot_base: 0,
            delays: Vec::new(),
            extra: Vec::new(),
        }
    }

    fn pin_at(node: usize, open: bool, open_high: bool) -> IcPin {
        IcPin { node, open, open_high, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    fn powered(v_gnd: f64, vs: f64) -> IcPower {
        IcPower { powered: true, v_gnd, vs, r_out: 156.25, vil: 0.8, vih: 2.0, schmitt: None }
    }

    #[test]
    fn pull_targets_high_and_low_ttl() {
        let ic = ic(true);
        let power = powered(0.0, 5.0);
        let open_prefix = fnv1a("u1:1b:"); // n=0.1151, HIGH
        let mut pin = pin_at(1, true, false);
        pin.floating_input = true;
        pin.open_hash = open_prefix;

        let (g, target) = pull(&ic, &power, &pin, 0.0).expect("pulls while powered, open, floating input");
        assert!((g - 1e-6).abs() < 1e-18, "TTL pull is 1/1MOhm");
        assert!((target - 5.0).abs() < 1e-12, "HIGH target is gnd + vS");
    }

    #[test]
    fn pull_targets_low_cmos() {
        let ic = ic(false);
        let power = powered(0.0, 5.0);
        let dead_prefix = fnv1a("u1:1b:undefined:"); // n=0.68912, CMOS LOW
        let mut pin = pin_at(1, true, false);
        pin.floating_input = true;
        pin.open_hash = dead_prefix; // pull always uses the open key, not the dead key

        let (g, target) = pull(&ic, &power, &pin, 0.0).expect("pulls while powered, open, floating input");
        assert!((g - 1e-7).abs() < 1e-19, "CMOS pull is 1/10MOhm");
        assert!((target - 0.0).abs() < 1e-12, "LOW target is gnd");
    }

    #[test]
    fn no_pull_while_unpowered() {
        let ic = ic(true);
        let mut power = powered(0.0, 5.0);
        power.powered = false;
        let mut pin = pin_at(1, true, false);
        pin.floating_input = true;
        assert_eq!(pull(&ic, &power, &pin, 0.0), None);
    }

    #[test]
    fn no_pull_when_open_high() {
        let ic = ic(true);
        let power = powered(0.0, 5.0);
        let mut pin = pin_at(1, true, true);
        pin.floating_input = true;
        assert_eq!(pull(&ic, &power, &pin, 0.0), None, "IC_OPEN_HIGH_PINS get no pull at all");
    }

    #[test]
    fn no_pull_when_not_floating_input_or_not_open() {
        let ic = ic(true);
        let power = powered(0.0, 5.0);
        let mut open_non_input = pin_at(1, true, false);
        open_non_input.floating_input = false;
        assert_eq!(pull(&ic, &power, &open_non_input, 0.0), None);

        let mut driven = pin_at(1, false, false);
        driven.floating_input = true;
        assert_eq!(pull(&ic, &power, &driven, 0.0), None);
    }
}
