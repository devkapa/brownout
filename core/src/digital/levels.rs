//! Pin reads (core/README.md, "Kernel semantics: the digital core (W1c)",
//! point 4; spec §1.4, `_logicHigh`/`_logicHighH`/`_schmittLogicHigh`
//! sim-engine.ts:5060-5139). Owned by W1c worker C1.

use crate::digital::floating;
use crate::digital::power::IcPower;
use crate::digital::{IcPin, IcRecord};

/// One pin's logic level, in classic's precedence order (spec §1.4,
/// `_logicPinMap` sim-engine.ts:5141-5165 dispatching to `_schmittLogicHigh`/
/// `_logicHigh`):
///
/// 1. An `OPEN_HIGH` pin that is `OPEN` reads 1 — ahead of everything else,
///    including Schmitt: classic tests this case before it branches on
///    `useSchmitt`.
/// 2. Otherwise, on a Schmitt part (`power.schmitt.is_some()` — classic's
///    `useSchmitt` is a whole-part flag from the catalog, applied to *every*
///    pin, not just the ones with hysteresis specs of their own): an open
///    pin reads the floating model on its open key; a driven pin compares
///    against the scaled thresholds and holds `*schmitt_state` in the dead
///    band, writing it back only on a change (classic's `commit = true`).
/// 3. Otherwise: an open pin reads the floating model on its open key;
///    a driven pin compares against `power.vil`/`power.vih` and reads the
///    floating model on the *dead* key in between.
///
/// `v_pin` is the pin's raw (absolute) node voltage; it is read relative to
/// `power.v_gnd` internally, exactly as classic reads `V(pin) - power.gnd`.
pub fn read(ic: &IcRecord, power: &IcPower, pin: &IcPin, v_pin: f64, t: f64, schmitt_state: &mut bool) -> bool {
    if pin.open_high && pin.open {
        return true;
    }

    if let Some((vt_minus, vt_plus)) = power.schmitt {
        if pin.open {
            return floating::floating_high(pin.open_hash, t, ic.ttl);
        }
        let v = v_pin - power.v_gnd;
        let next = if v >= vt_plus {
            true
        } else if v <= vt_minus {
            false
        } else {
            *schmitt_state
        };
        if next != *schmitt_state {
            *schmitt_state = next;
        }
        return next;
    }

    if pin.open {
        return floating::floating_high(pin.open_hash, t, ic.ttl);
    }

    let v = v_pin - power.v_gnd;
    if v >= power.vih {
        true
    } else if v <= power.vil {
        false
    } else {
        floating::floating_high(pin.dead_hash, t, ic.ttl)
    }
}

/// The absolute node voltages at which this pin's [`read`] can change —
/// crossing watches for the solver (spec §1.4: "A reader ... registers that
/// pin's live thresholds ... with the partition's crossing watch"). Empty
/// for an open pin, since an open pin's read never depends on its node
/// voltage at all (it is either forced by `OPEN_HIGH`, or purely the
/// floating model). `ic` takes no part here: whether this pin is
/// Schmitt-governed is already folded into `power.schmitt` (a whole-part
/// flag, see [`read`]).
pub fn watch_thresholds(_ic: &IcRecord, power: &IcPower, pin: &IcPin) -> Vec<f64> {
    if pin.open {
        return Vec::new();
    }
    match power.schmitt {
        Some((vt_minus, vt_plus)) => vec![power.v_gnd + vt_minus, power.v_gnd + vt_plus],
        None => vec![power.v_gnd + power.vil, power.v_gnd + power.vih],
    }
}

/// `minV*0.9`/`maxV*1.1` on the raw supply difference — the powered window
/// `power::ic_power` gates on (spec §1.3). Split out here, over the IC
/// record alone, so a partition can register the crossing watch on its
/// `vcc`/`gnd` nodes without first computing a full [`IcPower`].
pub fn power_window(ic: &IcRecord) -> (f64, f64) {
    (ic.vcc_min * 0.9, ic.vcc_max * 1.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ic_with(ttl: bool, schmitt_unscaled: Option<(f64, f64)>) -> IcRecord {
        IcRecord {
            kind: 50,
            vcc: pin(1, false, false),
            gnd: pin(0, false, false),
            logic: Vec::new(),
            vcc_min: 4.75,
            vcc_max: 5.25,
            ttl,
            cmos_ratio: false,
            vil: Some(0.8),
            vih: Some(2.0),
            schmitt: schmitt_unscaled,
            nominal: 5.0,
            io_max: Some(0.008),
            delay: 20e-9,
            slot_base: 0,
            delays: Vec::new(),
            extra: Vec::new(),
        }
    }

    fn pin(node: usize, open: bool, open_high: bool) -> IcPin {
        IcPin { node, open, open_high, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    fn power_plain(v_gnd: f64, vs: f64) -> IcPower {
        IcPower { powered: true, v_gnd, vs, r_out: 156.25, vil: 0.8, vih: 2.0, schmitt: None }
    }

    fn power_schmitt(v_gnd: f64, vs: f64, vt_minus: f64, vt_plus: f64) -> IcPower {
        IcPower { powered: true, v_gnd, vs, r_out: 62.5, vil: 0.8, vih: 2.0, schmitt: Some((vt_minus, vt_plus)) }
    }

    #[test]
    fn open_high_pin_that_is_open_reads_high_even_on_a_schmitt_part() {
        let ic = ic_with(true, Some((2.7, 1.6)));
        let power = power_schmitt(0.0, 5.0, 1.6, 2.7);
        let p = pin(1, true, true);
        let mut state = false;
        assert!(read(&ic, &power, &p, 0.0, 0.0, &mut state));
        assert!(!state, "open-high never touches schmitt state");
    }

    #[test]
    fn plain_thresholds_high_low_and_dead_band() {
        let ic = ic_with(true, None);
        let power = power_plain(0.0, 5.0); // vil=0.8, vih=2.0
        let mut state = false;

        assert!(read(&ic, &power, &pin(1, false, false), 2.0, 0.0, &mut state), "v == vih is HIGH");
        assert!(!read(&ic, &power, &pin(1, false, false), 0.8, 0.0, &mut state), "v == vil is LOW");
    }

    #[test]
    fn dead_band_uses_the_dead_key_not_the_open_key() {
        // Golden vector "u1:1b:undefined:0": n=0.68912 -> CMOS LOW, TTL HIGH.
        let dead_key_prefix = {
            // Build the ":undefined:" prefix state the same way the compiler
            // does, so this test exercises the real dead_hash field, not a
            // hand-picked number.
            "u1:1b:undefined:".bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193))
        };
        let mut ic = ic_with(false, None); // CMOS: dead-band verdict is LOW
        ic.vil = Some(0.8);
        ic.vih = Some(2.0);
        let power = power_plain(0.0, 5.0);
        let mut p = pin(1, false, false);
        p.dead_hash = dead_key_prefix;
        p.open_hash = 0xdead_beef; // must not be consulted for a driven pin
        let mut state = false;

        assert!(!read(&ic, &power, &p, 1.4, 0.0, &mut state), "v=1.4 is in [vil, vih]: CMOS reads LOW");
    }

    #[test]
    fn open_pin_uses_the_open_key_not_the_dead_key() {
        let open_key_prefix = {
            "u1:1b:".bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193))
        };
        let ic = ic_with(true, None); // TTL: open-key vector 0 is HIGH
        let power = power_plain(0.0, 5.0);
        let mut p = pin(1, true, false);
        p.open_hash = open_key_prefix;
        p.dead_hash = 0xdead_beef; // must not be consulted for an open pin
        let mut state = false;

        assert!(read(&ic, &power, &p, 0.0, 0.0, &mut state));
    }

    #[test]
    fn schmitt_holds_state_in_the_band_and_writes_only_on_change() {
        let ic = ic_with(true, Some((2.7, 1.6)));
        let power = power_schmitt(0.0, 5.0, 1.6, 2.7); // already scaled: 1.6/2.7
        let mut state = false; // starts LOW

        // In the dead band (1.6 < v < 2.7): holds the previous state.
        assert!(!read(&ic, &power, &pin(1, false, false), 2.0, 0.0, &mut state));
        assert!(!state);

        // Cross above vtPlus: commits HIGH.
        assert!(read(&ic, &power, &pin(1, false, false), 3.0, 0.0, &mut state));
        assert!(state);

        // Back in the dead band: holds the now-HIGH state (hysteresis).
        assert!(read(&ic, &power, &pin(1, false, false), 2.0, 0.0, &mut state));
        assert!(state);

        // Cross below vtMinus: commits LOW.
        assert!(!read(&ic, &power, &pin(1, false, false), 1.0, 0.0, &mut state));
        assert!(!state);
    }

    #[test]
    fn schmitt_open_pin_reads_the_open_key_and_never_touches_state() {
        let open_key_prefix = {
            "u1:1b:".bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193))
        };
        let ic = ic_with(true, Some((2.7, 1.6))); // TTL: open-key vector 0 is HIGH
        let power = power_schmitt(0.0, 5.0, 1.6, 2.7);
        let mut p = pin(1, true, false);
        p.open_hash = open_key_prefix;
        let mut state = false;

        assert!(read(&ic, &power, &p, 0.0, 0.0, &mut state));
        assert!(!state, "the open branch never reads or writes schmitt_state");
    }

    #[test]
    fn watch_thresholds_empty_for_open_pin() {
        let ic = ic_with(true, None);
        let power = power_plain(0.0, 5.0);
        assert!(watch_thresholds(&ic, &power, &pin(1, true, false)).is_empty());
    }

    #[test]
    fn watch_thresholds_plain() {
        let ic = ic_with(true, None);
        let power = power_plain(1.5, 5.0); // vil=0.8, vih=2.0, gnd offset 1.5
        let watches = watch_thresholds(&ic, &power, &pin(1, false, false));
        assert_eq!(watches, vec![1.5 + 0.8, 1.5 + 2.0]);
    }

    #[test]
    fn watch_thresholds_schmitt() {
        let ic = ic_with(true, Some((2.7, 1.6)));
        let power = power_schmitt(1.5, 5.0, 1.6, 2.7);
        let watches = watch_thresholds(&ic, &power, &pin(1, false, false));
        assert_eq!(watches, vec![1.5 + 1.6, 1.5 + 2.7]);
    }

    #[test]
    fn power_window_edges() {
        // LS: 4.75/5.25 -> 4.275/5.775.
        let mut ls = ic_with(true, None);
        ls.vcc_min = 4.75;
        ls.vcc_max = 5.25;
        let (lo, hi) = power_window(&ls);
        assert!((lo - 4.275).abs() < 1e-12);
        assert!((hi - 5.775).abs() < 1e-12);

        // HC: 2.0/6.0 -> 1.8/6.6.
        let mut hc = ic_with(true, None);
        hc.vcc_min = 2.0;
        hc.vcc_max = 6.0;
        let (lo, hi) = power_window(&hc);
        assert!((lo - 1.8).abs() < 1e-12);
        assert!((hi - 6.6).abs() < 1e-12);

        // CMOS-4000: 3.0/15.0 -> 2.7/16.5.
        let mut cmos = ic_with(true, None);
        cmos.vcc_min = 3.0;
        cmos.vcc_max = 15.0;
        let (lo, hi) = power_window(&cmos);
        assert!((lo - 2.7).abs() < 1e-12);
        assert!((hi - 16.5).abs() < 1e-12);
    }
}
