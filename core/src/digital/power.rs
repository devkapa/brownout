//! IC power derivation (core/README.md, "Kernel semantics: the digital core
//! (W1c)", point 1; spec §1.3, `_icPowerInfo` sim-engine.ts:4977-5027,
//! `thresholdsFor` sim-engine.ts:1362-1377). Owned by W1c worker C1.
//!
//! This is `_icPowerInfo`'s **normal path** only (sim-engine.ts:4997-5026).
//! Its no-supply-pin fallback (`:4982-4995`, a part with no catalog `vcc`/
//! `gnd` pin, or a stale/missing catalog entry) never reaches Rust: the
//! compiler declines any circuit whose IC supply pins don't both resolve
//! (core/README.md's record-layout table, "The compiler declines an IC..."),
//! so every [`IcRecord`] that reaches this module always carries real `vcc`
//! and `gnd` pins.

use crate::digital::IcRecord;

/// One IC's power state and input thresholds at one instant, derived purely
/// from its two supply-node voltages. Classic recomputes `_icPowerInfo` from
/// every Newton iterate with no Jacobian term for how it depends on the
/// supply (spec §1.3, contract point 3); `ic_power` mirrors that by being a
/// pure, cheap-to-call function rather than something cached on the record.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IcPower {
    pub powered: bool,
    /// The raw `gnd` node voltage (not assumed to be 0), so a pin read can
    /// be expressed relative to it exactly as classic's `power.gnd` is.
    pub v_gnd: f64,
    /// `max(0, vSraw)` — classic's `vSupply` field, after the clamp that
    /// keeps a reversed or unpowered supply from going negative downstream.
    pub vs: f64,
    pub r_out: f64,
    pub vil: f64,
    pub vih: f64,
    /// `(vtMinus*s, vtPlus*s)` for a Schmitt part (`s` below), `None`
    /// otherwise.
    pub schmitt: Option<(f64, f64)>,
}

/// The output-stage driver resistance floor, and the value used whenever
/// there is no `io_max` to derive one from (spec §1.3: `GATE_R_OUT`,
/// sim-engine.ts:985).
const GATE_R_OUT: f64 = 50.0;

/// Whether the IC is powered at these supply voltages: [`ic_power`]'s
/// `powered` field, on its own. A caller that only needs the verdict (the
/// digital-slot readout asks it of every IC at every completed instant) skips
/// the thresholds and the output resistance `ic_power` also derives;
/// `ic_power` itself calls this, so the two cannot drift apart.
pub fn powered(ic: &IcRecord, v_vcc: f64, v_gnd: f64) -> bool {
    let v_supply_raw = v_vcc - v_gnd;
    !ic.vcc.open && !ic.gnd.open && v_supply_raw >= ic.vcc_min * 0.9 && v_supply_raw <= ic.vcc_max * 1.1
}

/// What a power hold (W1c point 1a, classic's H8) makes of an IC's verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HoldKind {
    /// No hold: the verdict is the window test ([`powered`]).
    #[default]
    Free,
    /// Held unpowered: the IC starves its own supply when powered.
    Off,
    /// A probe solve's fixed verdict (the hold procedure's steps 1 and 3).
    Probe(bool),
}

impl HoldKind {
    /// A byte naming the kind, for a cache key or a memo.
    pub fn tag(self) -> u8 {
        match self {
            HoldKind::Free => 0,
            HoldKind::Off => 1,
            HoldKind::Probe(false) => 2,
            HoldKind::Probe(true) => 3,
        }
    }
}

/// Where a hold ends (W1c point 1a): when the supply difference, the IC
/// still off, passes `level` in the direction `rising` names (inward of the
/// window edge it starved at, by its own drop plus `margin`: two reached
/// widths, doubled at each retry that still starves past it, ruling R51
/// (b)). `since` is the instant the hold was taken or last moved: the next
/// retry comes at a later one, as classic's comes at its next host step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Release {
    pub level: f64,
    pub rising: bool,
    pub margin: f64,
    pub since: f64,
}

/// One IC's entry in the solver's hold table. `Default` is free.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PowerHold {
    pub kind: HoldKind,
    pub release: Option<Release>,
}

impl PowerHold {
    pub const FREE: PowerHold = PowerHold { kind: HoldKind::Free, release: None };

    pub fn off() -> PowerHold {
        PowerHold { kind: HoldKind::Off, release: None }
    }

    pub fn is_free(&self) -> bool {
        self.kind == HoldKind::Free
    }
}

/// The hold of the IC at device index `device` in a device-indexed table.
/// The solver hands out an empty table while no IC is held, so the common
/// case is one bounds check.
#[inline]
pub fn hold_of(holds: &[PowerHold], device: usize) -> PowerHold {
    holds.get(device).copied().unwrap_or(PowerHold::FREE)
}

/// The power verdict, the one function every site that decides whether an
/// IC is powered goes through: the hold's when there is one, else the
/// window test.
#[inline]
pub fn verdict(ic: &IcRecord, v_vcc: f64, v_gnd: f64, hold: PowerHold) -> bool {
    match hold.kind {
        HoldKind::Free => powered(ic, v_vcc, v_gnd),
        HoldKind::Off | HoldKind::Probe(false) => false,
        HoldKind::Probe(true) => true,
    }
}

/// [`ic_power`] with `powered` from [`verdict`]: a held-off IC is unpowered
/// for every purpose, its thresholds and output resistance unused.
#[inline]
pub fn ic_power_held(ic: &IcRecord, v_vcc: f64, v_gnd: f64, hold: PowerHold) -> IcPower {
    let mut power = ic_power(ic, v_vcc, v_gnd);
    if !hold.is_free() {
        power.powered = verdict(ic, v_vcc, v_gnd, hold);
    }
    power
}

/// `_icPowerInfo`, normal path (spec §1.3). `v_vcc` and `v_gnd` are the raw
/// solved (or Newton-iterate) node voltages at the IC's `vcc`/`gnd` pins —
/// callers read them off whatever voltage vector the pins' nodes sit in;
/// this function does no node lookups of its own. This is the free verdict:
/// a site that can see the hold table calls [`ic_power_held`].
pub fn ic_power(ic: &IcRecord, v_vcc: f64, v_gnd: f64) -> IcPower {
    // The gate is tested on the raw, unclamped difference (sim-engine.ts
    // :5001, :5005-5008): a reversed supply simply reads as unpowered, with
    // no separate reverse-polarity mode. `minV*0.9`/`maxV*1.1` are computed
    // in f64 at run time, never folded into a decimal constant, per the
    // contract's explicit note (spec §1.3).
    let v_supply_raw = v_vcc - v_gnd;
    let powered = powered(ic, v_vcc, v_gnd);
    let vs = v_supply_raw.max(0.0);

    // `ic.io_max` is already `None` unless the compiler resolved
    // `specs.io_max > 0` (core/README.md's IC-params list, param 11), so the
    // only remaining condition here is `vs > 0`.
    let r_out = match ic.io_max {
        Some(io_max) if vs > 0.0 => (vs / (4.0 * io_max)).clamp(GATE_R_OUT, 1000.0),
        _ => GATE_R_OUT,
    };

    let (vil, vih) = thresholds_for(ic, vs);

    // The Schmitt pair ships from the compiler unscaled (`IcRecord.schmitt`
    // is `(vtPlus, vtMinus)` at the catalog's nominal supply); scale by
    // `s = nominal>0 ? vs/nominal : 1` here so every reader downstream
    // (`levels::read`, `levels::watch_thresholds`) works with absolute
    // volts, never re-deriving `s`. Note the tuple order flips to
    // `(vtMinus*s, vtPlus*s)` — the locked field order.
    let schmitt = ic.schmitt.map(|(vt_plus, vt_minus)| {
        let s = if ic.nominal > 0.0 { vs / ic.nominal } else { 1.0 };
        (vt_minus * s, vt_plus * s)
    });

    IcPower { powered, v_gnd, vs, r_out, vil, vih, schmitt }
}

/// A driver conductance as a function of the supply difference `vs`, of the
/// form `alpha + beta / vs` on each of three pieces split at `kinks` (ascending, either
/// may be infinite), the form every output stage's conductance takes. The
/// curvature band reads its derivatives (Rails point 3: a driver's `R_out`
/// moves with a rail supply).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupplyCurve {
    pub kinks: [f64; 2],
    pub pieces: [(f64, f64); 3],
}

impl SupplyCurve {
    pub fn constant(g: f64) -> SupplyCurve {
        SupplyCurve { kinks: [f64::INFINITY; 2], pieces: [(g, 0.0); 3] }
    }

    /// `(alpha, beta)` of the piece holding `vs` (a kink belongs to the
    /// piece below it, as the clamps that make it do).
    pub fn piece(&self, vs: f64) -> (f64, f64) {
        if vs <= self.kinks[0] {
            self.pieces[0]
        } else if vs <= self.kinks[1] {
            self.pieces[1]
        } else {
            self.pieces[2]
        }
    }

    /// The conductance and its first three derivatives in `vs`. A constant
    /// piece's are 0 at any supply: written `-b / vs^2` and so on, they were
    /// 0 / 0 at `vs = 0`, where a gate whose power is judged at the loop's
    /// rail (`Bnd::hold_power_at`) stamps its constant `R_out` piece while a
    /// CC seed solving the rail below the gate's ground reads its supply
    /// difference as 0 (cold 230 tpl 2: a NaN rail sensitivity, `-4`).
    /// Elsewhere the bits are the formula's.
    pub fn derivs(&self, vs: f64) -> [f64; 4] {
        let (a, b) = self.piece(vs);
        if b == 0.0 && vs == 0.0 {
            return [a, -0.0, 0.0, -0.0];
        }
        [a + b / vs, -b / (vs * vs), 2.0 * b / (vs * vs * vs), -6.0 * b / (vs * vs * vs * vs)]
    }
}

/// `1 / ic_power(..).r_out` as a [`SupplyCurve`]: `vs / (4 io_max)` clamped
/// to `[GATE_R_OUT, 1000]`, or `GATE_R_OUT` with no `io_max`.
pub fn r_out_curve(ic: &IcRecord) -> SupplyCurve {
    match ic.io_max {
        Some(io_max) => SupplyCurve { kinks: [4.0 * io_max * GATE_R_OUT, 4.0 * io_max * 1000.0], pieces: [(1.0 / GATE_R_OUT, 0.0), (0.0, 4.0 * io_max), (1.0 / 1000.0, 0.0)] },
        None => SupplyCurve::constant(1.0 / GATE_R_OUT),
    }
}

/// `thresholdsFor` (spec §1.3, sim-engine.ts:1362-1377). `vil` and `vih` are
/// resolved independently, exactly as classic's two `??` fallbacks are
/// independent: a CMOS-ratio part ignores both fixed catalog values, but
/// otherwise each threshold uses its own fixed value when the compiler
/// marked it so (`IcRecord.vil`/`vih` is `Some` — README's IC-params list,
/// params 3-6) and falls back to the shared generic formula when not,
/// whether or not the *other* threshold happened to be fixed.
fn thresholds_for(ic: &IcRecord, vs: f64) -> (f64, f64) {
    if ic.cmos_ratio {
        return (f64::max(0.2, 0.3 * vs), f64::max(0.4, 0.7 * vs));
    }
    let vil = ic.vil.unwrap_or_else(|| f64::max(0.8, 0.3 * vs));
    let vih = ic.vih.unwrap_or_else(|| f64::max(2.0, 0.6 * vs));
    (vil, vih)
}

/// The supply differences `vs` at which [`thresholds_for`]'s clamps start
/// to scale: past them each generic threshold is affine in `vs`, and a
/// crossing solved under a moving supply must not step across one (Rails
/// point 4). Empty when both levels are fixed.
pub fn threshold_kinks(ic: &IcRecord) -> Vec<f64> {
    if ic.cmos_ratio {
        return vec![0.2 / 0.3, 0.4 / 0.7];
    }
    let mut kinks = Vec::new();
    if ic.vil.is_none() {
        kinks.push(0.8 / 0.3);
    }
    if ic.vih.is_none() {
        kinks.push(2.0 / 0.6);
    }
    kinks
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::digital::IcPin;

    fn open_pin(node: usize) -> IcPin {
        IcPin { node, open: false, open_high: false, open_hash: 0, dead_hash: 0, floating_input: false, open_collector: false }
    }

    /// A bare-minimum record with only the fields `ic_power`/`thresholds_for`
    /// read; callers override what a given test cares about.
    fn base_ic() -> IcRecord {
        IcRecord {
            kind: 50,
            vcc: open_pin(1),
            gnd: open_pin(0),
            logic: Vec::new(),
            vcc_min: 4.75,
            vcc_max: 5.25,
            ttl: true,
            cmos_ratio: false,
            vil: None,
            vih: None,
            schmitt: None,
            nominal: 5.0,
            io_max: None,
            delay: 20e-9,
            slot_base: 0,
            delays: Vec::new(),
            extra: Vec::new(),
        }
    }

    #[test]
    fn the_r_out_curve_is_the_conductance_ic_power_stamps() {
        // The curvature band reads R_out's derivatives off this curve; it is
        // the same function as `ic_power`'s on every piece and across both
        // clamps, and its derivatives are the curve's own.
        for io_max in [None, Some(0.008), Some(1e-3), Some(0.025)] {
            let ic = IcRecord { io_max, vcc_min: 0.0, vcc_max: 100.0, ..base_ic() };
            let curve = r_out_curve(&ic);
            for k in 1..=4000 {
                let vs = 0.01 * k as f64;
                let want = 1.0 / ic_power(&ic, vs, 0.0).r_out;
                let [g, dg, ..] = curve.derivs(vs);
                assert!((g - want).abs() <= 1e-14 * want, "io_max {io_max:?} vs {vs}: {g} against {want}");
                let h = 1e-6;
                if curve.kinks.iter().all(|k| (k - vs).abs() > 2.0 * h) {
                    let fd = (curve.derivs(vs + h)[0] - curve.derivs(vs - h)[0]) / (2.0 * h);
                    assert!((dg - fd).abs() <= 1e-6 * want, "io_max {io_max:?} vs {vs}: slope {dg} against {fd}");
                }
            }
        }
    }

    #[test]
    fn powered_window_ls() {
        // LS-TTL: [0.9*4.75, 1.1*5.25] = [4.275, 5.775] (spec §1.3 table).
        let ic = base_ic();
        assert!(!ic_power(&ic, 4.274, 0.0).powered, "just under the floor");
        assert!(ic_power(&ic, 4.275, 0.0).powered, "at the floor");
        assert!(ic_power(&ic, 5.775, 0.0).powered, "at the ceiling");
        assert!(!ic_power(&ic, 5.776, 0.0).powered, "just over the ceiling");
    }

    #[test]
    fn powered_window_hc() {
        // HC-CMOS: vcc_range [2.0, 6.0] -> [1.8, 6.6].
        let mut ic = base_ic();
        ic.vcc_min = 2.0;
        ic.vcc_max = 6.0;
        assert!(!ic_power(&ic, 1.799, 0.0).powered);
        assert!(ic_power(&ic, 1.8, 0.0).powered);
        assert!(ic_power(&ic, 6.6, 0.0).powered);
        assert!(!ic_power(&ic, 6.601, 0.0).powered);
    }

    #[test]
    fn powered_window_cmos_4000() {
        // CMOS-4000: vcc_range [3.0, 15.0] -> [2.7, 16.5].
        let mut ic = base_ic();
        ic.vcc_min = 3.0;
        ic.vcc_max = 15.0;
        assert!(!ic_power(&ic, 2.699, 0.0).powered);
        assert!(ic_power(&ic, 2.7, 0.0).powered);
        assert!(ic_power(&ic, 16.5, 0.0).powered);
        assert!(!ic_power(&ic, 16.501, 0.0).powered);
    }

    #[test]
    fn unpowered_when_a_supply_pin_is_open() {
        let mut ic = base_ic();
        ic.vcc.open = true;
        assert!(!ic_power(&ic, 5.0, 0.0).powered, "vcc open");
        let mut ic2 = base_ic();
        ic2.gnd.open = true;
        assert!(!ic_power(&ic2, 5.0, 0.0).powered, "gnd open");
    }

    #[test]
    fn r_out_at_5v() {
        // LS-TTL family (74ls00/04/08/32/86/157/283/161/173/189): io_max
        // 0.008 -> 156.25 ohm.
        let mut ic = base_ic();
        ic.io_max = Some(0.008);
        assert!((ic_power(&ic, 5.0, 0.0).r_out - 156.25).abs() < 1e-9);

        // 74ls245/74ls47: io_max 0.024 -> 52.083... ohm.
        ic.io_max = Some(0.024);
        assert!((ic_power(&ic, 5.0, 0.0).r_out - 52.083_333_333_333_33).abs() < 1e-9);

        // 74hc14: io_max 0.02 -> 62.5 ohm.
        ic.io_max = Some(0.02);
        assert!((ic_power(&ic, 5.0, 0.0).r_out - 62.5).abs() < 1e-9);

        // 74hc138: io_max 0.025 -> exactly the 50 ohm floor.
        ic.io_max = Some(0.025);
        assert!((ic_power(&ic, 5.0, 0.0).r_out - 50.0).abs() < 1e-9);
    }

    #[test]
    fn r_out_is_the_floor_with_no_io_max_or_no_supply() {
        let mut ic = base_ic();
        ic.io_max = None;
        assert_eq!(ic_power(&ic, 5.0, 0.0).r_out, 50.0);

        ic.io_max = Some(0.008);
        // vs == 0 (shorted supply): the `vs > 0` guard falls through to the
        // floor rather than dividing by zero volts.
        assert_eq!(ic_power(&ic, 0.0, 0.0).r_out, 50.0);
    }

    #[test]
    fn r_out_clamps_to_1000_at_high_supply() {
        let mut ic = base_ic();
        ic.vcc_max = 20.0; // just so a high vSupply is still "powered" for the test's own sanity, unused by r_out itself
        ic.io_max = Some(1e-6); // tiny io_max drives vs/(4*io_max) far past 1000
        assert_eq!(ic_power(&ic, 5.0, 0.0).r_out, 1000.0);
    }

    #[test]
    fn thresholds_fixed_ls() {
        // LS: vilFixed/vihFixed from the catalog, 0.8/2.0, regardless of vS.
        let mut ic = base_ic();
        ic.vil = Some(0.8);
        ic.vih = Some(2.0);
        let power = ic_power(&ic, 5.0, 0.0);
        assert_eq!((power.vil, power.vih), (0.8, 2.0));
    }

    #[test]
    fn thresholds_hc_ratio_at_5v_and_3v3() {
        let mut ic = base_ic();
        ic.cmos_ratio = true;
        let at5 = ic_power(&ic, 5.0, 0.0);
        assert!((at5.vil - 1.5).abs() < 1e-12, "max(0.2, 0.3*5) = 1.5");
        assert!((at5.vih - 3.5).abs() < 1e-12, "max(0.4, 0.7*5) = 3.5");

        let at3v3 = ic_power(&ic, 3.3, 0.0);
        assert!((at3v3.vil - 0.99).abs() < 1e-12, "max(0.2, 0.3*3.3) = 0.99");
        assert!((at3v3.vih - 2.31).abs() < 1e-12, "max(0.4, 0.7*3.3) = 2.31");
    }

    #[test]
    fn thresholds_generic_fallback() {
        // No fixed catalog value and no CMOS ratio: max(0.8, 0.3*vS) /
        // max(2.0, 0.6*vS). The floor wins below vS = 0.8/0.3 (vil) and
        // 2.0/0.6 (vih); the ratio wins above.
        let ic = base_ic(); // vil/vih both None, cmos_ratio false
        let at2 = ic_power(&ic, 2.0, 0.0);
        assert_eq!((at2.vil, at2.vih), (0.8, 2.0), "0.3*2=0.6 < 0.8; 0.6*2=1.2 < 2.0: both floors win");

        let at12 = ic_power(&ic, 12.0, 0.0);
        assert!((at12.vil - 3.6).abs() < 1e-12, "0.3*12 = 3.6 > 0.8");
        assert!((at12.vih - 7.2).abs() < 1e-12, "0.6*12 = 7.2 > 2.0");
    }

    #[test]
    fn schmitt_band_74hc14_at_5v_and_3v3() {
        // 74hc14: vtPlus=2.7, vtMinus=1.6, nominal=5.
        let mut ic = base_ic();
        ic.schmitt = Some((2.7, 1.6));
        ic.nominal = 5.0;

        let at5 = ic_power(&ic, 5.0, 0.0).schmitt.expect("schmitt part");
        assert!((at5.0 - 1.6).abs() < 1e-12, "vtMinus*1 = 1.6");
        assert!((at5.1 - 2.7).abs() < 1e-12, "vtPlus*1 = 2.7");

        let at3v3 = ic_power(&ic, 3.3, 0.0).schmitt.expect("schmitt part");
        let s = 3.3 / 5.0;
        assert!((at3v3.0 - 1.6 * s).abs() < 1e-12);
        assert!((at3v3.1 - 2.7 * s).abs() < 1e-12);
    }

    #[test]
    fn the_verdict_is_the_holds_and_the_window_tests_only_while_free() {
        let ic = base_ic();
        let probe = |b| PowerHold { kind: HoldKind::Probe(b), release: None };
        for (v, inside) in [(5.0, true), (4.0, false), (6.0, false)] {
            assert_eq!(verdict(&ic, v, 0.0, PowerHold::FREE), inside);
            assert!(!verdict(&ic, v, 0.0, PowerHold::off()));
            assert!(!verdict(&ic, v, 0.0, probe(false)));
            assert!(verdict(&ic, v, 0.0, probe(true)));
            assert_eq!(ic_power_held(&ic, v, 0.0, PowerHold::FREE), ic_power(&ic, v, 0.0));
            assert!(!ic_power_held(&ic, v, 0.0, PowerHold::off()).powered);
        }
        assert_eq!(hold_of(&[], 3), PowerHold::FREE, "an empty table holds nothing");
        assert_eq!(hold_of(&[PowerHold::FREE, PowerHold::off()], 1), PowerHold::off());
    }

    #[test]
    fn gnd_is_carried_through_raw() {
        let ic = base_ic();
        let power = ic_power(&ic, 8.0, 3.2);
        assert_eq!(power.v_gnd, 3.2);
        assert!((power.vs - 4.8).abs() < 1e-12, "vSraw = 8.0 - 3.2 = 4.8");
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for IcPower {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.powered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.v_gnd, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vs, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.r_out, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vil, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.vih, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.schmitt, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for HoldKind {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            HoldKind::Free => 0,
            HoldKind::Off => 0,
            HoldKind::Probe(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
        }
    }
}

impl crate::snapshot::HeapBytes for Release {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.level, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rising, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.margin, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.since, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for PowerHold {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kind, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.release, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for SupplyCurve {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.kinks, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pieces, w); }
        n
    }
}
