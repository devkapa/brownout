//! The 11 W1c combinational-IC truth tables (core/README.md, "Kernel
//! semantics: the digital core (W1c)", point 7; `core/spec/w1-classic-
//! semantics.md` §3.1, §3.6). Owned by W1c worker C2.
//!
//! `evaluate` mirrors classic's `evalCombinationalIC`
//! (`src/sim/engine/digital.ts`) plus the engine-level `/oe` gate
//! `_combinationalOutputs` (`src/sim/engine/sim-engine.ts:5206-5227`) applies
//! *before* calling it — the reason a 157/245 held at `/oe = 1` produces no
//! outputs at all here, never `evalCombinationalIC`'s own (dead, in classic)
//! `oe ? false : ...` branch. `levels` and the return value share one
//! indexing convention throughout this file:
//! - `levels[i]` is logic pin `i`'s read, in `icPinMap` order (README's "IC
//!   logic pins" table) — every logic pin, inputs and outputs and controls
//!   alike, because that is what classic's own pin map carries and what
//!   `combinational_golden.rs`'s bit pattern packs one bit per.
//! - the returned `Vec`'s entry `i` is [`output_capable`]`(kind)[i]`'s
//!   result, not `levels[i]`'s — a different, shorter indexing, over only
//!   the pins that own a digital slot.
//!
//! Every level in and out is the ELECTRICAL convention (`true` = HIGH /
//! released), matching the locked API in core/README.md: classic inverts
//! 74LS47's sinking-truth-table bits itself before returning
//! (`digital.ts:583-592`), and this module does the same rather than
//! exposing the sinking convention anywhere.

use crate::kinds;

// Pulls in the generated golden table (`scripts/gen-ic-truth-golden.ts`) as a
// child module of this one, from its sibling file — not declared in mod.rs
// (orchestrator-owned) because nothing outside this file's own tests needs
// it. `cfg(test)`-only: the table exists to replay through `evaluate()` in
// `cargo test`, never to ship in the release wasm, so it must not count
// toward that build's size or trip its dead-code warning.
#[cfg(test)]
#[path = "combinational_golden.rs"]
mod combinational_golden;

/// How many logic pins (after `vcc`/`gnd`) kind `kind` carries — `L` in
/// core/README.md's "IC logic pins" table. `0` for a kind this module does
/// not own; every caller already gates on [`super::is_combinational_ic`]
/// before reaching here, so that arm is unreachable in practice, not a
/// documented "unknown kind" outcome.
pub fn logic_pin_count(kind: u32) -> usize {
    match kind {
        kinds::IC_74LS00 | kinds::IC_74LS08 | kinds::IC_74LS32 | kinds::IC_74LS86 => 12,
        kinds::IC_74LS04 | kinds::IC_74HC14 => 12,
        kinds::IC_74LS157 => 14,
        kinds::IC_74LS283 => 14,
        kinds::IC_74HC138 => 14,
        kinds::IC_74LS47 => 14,
        kinds::IC_74LS245 => 18,
        _ => 0,
    }
}

/// Indices into the logic pins (in pin order) that own a digital slot: the
/// `o` pins, or, for the 74ls245, all 16 `a`/`b` pins (README's locked API —
/// the evaluator picks which side of each pair is driven, so both sides are
/// "output-capable" even though only one is driven on any given evaluation).
/// `&[]` for a kind this module does not own (see [`logic_pin_count`]'s doc).
///
/// The table itself moved to `digital::output_capable` (W1d): every W1d kind
/// needs the identical lookup, so it lives once at the shared parent and
/// this forwards to it, keeping this module's own (many) call sites and
/// tests compiling unchanged.
pub use super::output_capable;

/// Evaluates kind `kind`'s truth table over `levels` (one electrical read per
/// logic pin, `icPinMap` order — see the module doc). Returns one entry per
/// [`output_capable`]`(kind)` pin, `Some(level)` (electrical) when classic's
/// result carries that pin's key, `None` when it does not (a 157/245 tri-
/// stated by `/oe`, or the undriven side of a 245).
pub fn evaluate(kind: u32, levels: &[bool]) -> Vec<Option<bool>> {
    match kind {
        kinds::IC_74LS00 => gate3(levels, Gate::Nand, 4),
        kinds::IC_74LS08 => gate3(levels, Gate::And, 4),
        kinds::IC_74LS32 => gate3(levels, Gate::Or, 4),
        kinds::IC_74LS86 => gate3(levels, Gate::Xor, 4),
        kinds::IC_74LS04 | kinds::IC_74HC14 => gate2_not(levels, 6),
        kinds::IC_74LS157 => mux157(levels),
        kinds::IC_74LS283 => adder283(levels),
        kinds::IC_74HC138 => decoder138(levels),
        kinds::IC_74LS47 => bcd47(levels),
        kinds::IC_74LS245 => transceiver245(levels),
        _ => Vec::new(),
    }
}

#[derive(Clone, Copy)]
enum Gate {
    And,
    Or,
    Nand,
    Xor,
}

fn eval_gate(gate: Gate, a: bool, b: bool) -> bool {
    match gate {
        Gate::And => a && b,
        Gate::Or => a || b,
        Gate::Nand => !(a && b),
        Gate::Xor => a != b, // digital.ts:700 -- parity reduce, equal to a != b for 2 inputs
    }
}

/// `74ls00`/`74ls08`/`74ls32`/`74ls86`: `gates` groups of `(a, b, y)`,
/// `y = gate(a, b)`. Always fully driven; these kinds have no `/oe`.
fn gate3(levels: &[bool], gate: Gate, gates: usize) -> Vec<Option<bool>> {
    (0..gates).map(|g| Some(eval_gate(gate, levels[g * 3], levels[g * 3 + 1]))).collect()
}

/// `74ls04`/`74hc14`: `gates` groups of `(a, y)`, `y = !a`. 74hc14's Schmitt
/// hysteresis is applied upstream, before this function ever sees a level
/// (spec §3.1: "same topology as 74ls04; hysteresis applied upstream"), so
/// the truth table itself is a plain inverter for both kinds.
fn gate2_not(levels: &[bool], gates: usize) -> Vec<Option<bool>> {
    (0..gates).map(|g| Some(!levels[g * 2])).collect()
}

/// `74ls157`: pins `sel(0) /oe(1) 1a(2) 1b(3) 1y(4) ... 4a(11) 4b(12) 4y(13)`.
fn mux157(levels: &[bool]) -> Vec<Option<bool>> {
    let sel = levels[0];
    let oe = levels[1];
    // The engine-level gate (module doc; sim-engine.ts's own check ahead of
    // ever calling evalCombinationalIC): /oe HIGH means no outputs at all,
    // not each gate individually going `false` (digital.ts:499's `oe ? false
    // : ...` is unreachable in classic).
    if oe {
        return vec![None; 4];
    }
    (0..4)
        .map(|g| {
            let a = levels[2 + g * 3];
            let b = levels[3 + g * 3];
            Some(if sel { b } else { a })
        })
        .collect()
}

/// `74ls283`: pins `a1(0) b1(1) a2(2) b2(3) a3(4) b3(5) a4(6) b4(7) c0(8)`,
/// outputs `s1(9) s2(10) s3(11) s4(12) c4(13)`. Ripple sum from `c0`
/// (digital.ts:504-514); no `/oe`, always fully driven.
fn adder283(levels: &[bool]) -> Vec<Option<bool>> {
    let bit = |l: bool| u8::from(l);
    let mut carry = bit(levels[8]);
    let mut out = Vec::with_capacity(5);
    for i in 0..4 {
        let sum = bit(levels[i * 2]) + bit(levels[i * 2 + 1]) + carry;
        out.push(Some((sum & 1) == 1));
        carry = sum >> 1;
    }
    out.push(Some(carry == 1));
    out
}

/// `74hc138`: pins `a0(0) a1(1) a2(2) e1n(3) e2n(4) e3(5)`, outputs
/// `y0n(6)..y7n(13)`. Disabled drives every output HIGH, not hi-Z
/// (digital.ts:532-554); no `/oe`-style absence.
fn decoder138(levels: &[bool]) -> Vec<Option<bool>> {
    let enabled = !levels[3] && !levels[4] && levels[5];
    let sel = usize::from(levels[0]) | (usize::from(levels[1]) << 1) | (usize::from(levels[2]) << 2);
    (0..8).map(|i| Some(!enabled || sel != i)).collect()
}

/// `LS47_SEGMENTS` (digital.ts:444-477), sinking convention: `true` means the
/// segment output transistor sinks current (pin driven LOW). Row index is
/// the BCD value 0-15; rows 10-15 are the real 74LS47's partial glyphs, not
/// hex letters.
#[rustfmt::skip]
const LS47_SEGMENTS: [[bool; 7]; 16] = [
    // a      b      c      d      e      f      g
    [true,  true,  true,  true,  true,  true,  false], // 0
    [false, true,  true,  false, false, false, false], // 1
    [true,  true,  false, true,  true,  false, true ], // 2
    [true,  true,  true,  true,  false, false, true ], // 3
    [false, true,  true,  false, false, true,  true ], // 4
    [true,  false, true,  true,  false, true,  true ], // 5
    [false, false, true,  true,  true,  true,  true ], // 6
    [true,  true,  true,  false, false, false, false], // 7
    [true,  true,  true,  true,  true,  true,  true ], // 8
    [true,  true,  true,  false, false, true,  true ], // 9
    [false, false, false, true,  true,  false, true ], // 10
    [false, false, true,  true,  false, false, true ], // 11
    [false, true,  false, false, false, true,  true ], // 12
    [true,  false, false, true,  false, true,  true ], // 13
    [false, false, false, true,  true,  true,  true ], // 14
    [false, false, false, false, false, false, false], // 15
];

/// `74ls47`: pins `b(0) c(1) lt_n(2) rbo_n(3) rbi_n(4) d(5) a(6)`, outputs
/// `e_out(7) d_out(8) c_out(9) b_out(10) a_out(11) g_out(12) f_out(13)`.
/// Priority `rbo_n > lt_n > rbi_n` (spec §3.6); always fully driven -- the
/// 74ls47's open-collector-ness is an output-STAGE property (`mod.rs`'s
/// `open_collector` flag, stamped by the solver), not an absence here.
fn bcd47(levels: &[bool]) -> Vec<Option<bool>> {
    let rbo_low = !levels[3];
    let lt_low = !levels[2];
    let rbi_low = !levels[4];
    let bcd = usize::from(levels[6]) | (usize::from(levels[0]) << 1) | (usize::from(levels[1]) << 2) | (usize::from(levels[5]) << 3);

    let segs = if rbo_low {
        [false; 7]
    } else if lt_low {
        [true; 7]
    } else if rbi_low && bcd == 0 {
        [false; 7]
    } else {
        LS47_SEGMENTS[bcd]
    };

    // Electrical order is e,d,c,b,a,g,f (README's pin order) over the
    // sinking-convention `segs` indexed a,b,c,d,e,f,g; classic inverts
    // sinking -> electrical once (digital.ts:586-592) and so do we.
    [4usize, 3, 2, 1, 0, 6, 5].iter().map(|&i| Some(!segs[i])).collect()
}

/// `74ls245`: pins `dir(0) /oe(1) a1(2) b1(3) a2(4) b2(5) ... a8(16) b8(17)`.
/// Output order is every `a`/`b` pin, a-then-b per gate, matching
/// `output_capable`. `/oe` HIGH: nothing driven, either side (the same
/// engine-level gate as the 157, module doc). Otherwise `dir` picks which
/// side of each pair is driven from the other; the undriven side is `None`
/// (tri-stated, spec §3.6's table), never a stale level.
fn transceiver245(levels: &[bool]) -> Vec<Option<bool>> {
    let dir = levels[0];
    let oe = levels[1];
    let mut out = vec![None; 16];
    if oe {
        return out;
    }
    for g in 0..8 {
        let a_level = levels[2 + g * 2];
        let b_level = levels[3 + g * 2];
        if dir {
            out[g * 2 + 1] = Some(a_level); // b_i = a_i; a_i itself stays None
        } else {
            out[g * 2] = Some(b_level); // a_i = b_i; b_i itself stays None
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- logic_pin_count / output_capable against README's table ----------

    #[test]
    fn logic_pin_count_matches_the_readme_table() {
        assert_eq!(logic_pin_count(kinds::IC_74LS00), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS04), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS08), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS32), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS86), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS157), 14);
        assert_eq!(logic_pin_count(kinds::IC_74LS283), 14);
        assert_eq!(logic_pin_count(kinds::IC_74HC138), 14);
        assert_eq!(logic_pin_count(kinds::IC_74HC14), 12);
        assert_eq!(logic_pin_count(kinds::IC_74LS47), 14);
        assert_eq!(logic_pin_count(kinds::IC_74LS245), 18);
    }

    #[test]
    fn output_capable_matches_the_readme_table() {
        assert_eq!(output_capable(kinds::IC_74LS00), &[2, 5, 8, 11]);
        assert_eq!(output_capable(kinds::IC_74LS04), &[1, 3, 5, 7, 9, 11]);
        assert_eq!(output_capable(kinds::IC_74LS157), &[4, 7, 10, 13]);
        assert_eq!(output_capable(kinds::IC_74LS283), &[9, 10, 11, 12, 13]);
        assert_eq!(output_capable(kinds::IC_74HC138), &[6, 7, 8, 9, 10, 11, 12, 13]);
        assert_eq!(output_capable(kinds::IC_74LS47), &[7, 8, 9, 10, 11, 12, 13]);
        assert_eq!(output_capable(kinds::IC_74LS245).len(), 16);
        assert_eq!(output_capable(kinds::IC_74LS245)[0], 2, "a1 is the first output-capable pin");
        assert_eq!(output_capable(kinds::IC_74LS245)[15], 17, "b8 is the last");
    }

    #[test]
    fn every_output_capable_index_is_within_its_logic_pin_count() {
        for &kind in &[
            kinds::IC_74LS00,
            kinds::IC_74LS04,
            kinds::IC_74LS08,
            kinds::IC_74LS32,
            kinds::IC_74LS86,
            kinds::IC_74LS157,
            kinds::IC_74LS283,
            kinds::IC_74HC138,
            kinds::IC_74HC14,
            kinds::IC_74LS47,
            kinds::IC_74LS245,
        ] {
            let l = logic_pin_count(kind);
            for &i in output_capable(kind) {
                assert!(i < l, "kind {kind}: output_capable index {i} out of logic_pin_count {l}");
            }
        }
    }

    // ---- literal hand-written rows, one readable sanity check per kind ----
    // (the golden table below is the real coverage; these are a legibility
    // check that survives even if the golden file were ever regenerated
    // wrong in a way that still happened to replay clean.)

    #[test]
    fn nand_gate_0_of_4_is_nand_of_1a_1b() {
        // 1a=1, 1b=1 -> 1y=0; every other gate's inputs left LOW -> NAND=1.
        let mut levels = [false; 12];
        levels[0] = true; // 1a
        levels[1] = true; // 1b
        let out = evaluate(kinds::IC_74LS00, &levels);
        assert_eq!(out, vec![Some(false), Some(true), Some(true), Some(true)]);
    }

    #[test]
    fn hex_inverter_inverts_every_gate_independently() {
        let mut levels = [false; 12];
        levels[0] = true; // 1a HIGH -> 1y LOW
        levels[4] = true; // 3a HIGH -> 3y LOW
        let out = evaluate(kinds::IC_74LS04, &levels);
        assert_eq!(out, vec![Some(false), Some(true), Some(false), Some(true), Some(true), Some(true)]);
    }

    #[test]
    fn mux_157_selects_b_side_and_oe_high_mutes_every_output() {
        // sel=1 (B side), /oe=0 (enabled): 1a=0 1b=1 -> 1y=1; the rest LOW.
        let mut levels = [false; 14];
        levels[0] = true; // sel
        levels[3] = true; // 1b
        let enabled = evaluate(kinds::IC_74LS157, &levels);
        assert_eq!(enabled, vec![Some(true), Some(false), Some(false), Some(false)]);

        levels[1] = true; // /oe HIGH -> disabled regardless of sel/inputs
        let disabled = evaluate(kinds::IC_74LS157, &levels);
        assert_eq!(disabled, vec![None, None, None, None]);
    }

    #[test]
    fn adder_283_ripples_a_carry_through_all_four_bits() {
        // 1111 + 0001 + c0=0 = 10000: s=0000, c4=1.
        let mut levels = [false; 14];
        for i in 0..4 {
            levels[i * 2] = true; // a1..a4 = 1
        }
        levels[1] = true; // b1 = 1, b2..b4 = 0
        let out = evaluate(kinds::IC_74LS283, &levels);
        assert_eq!(out, vec![Some(false), Some(false), Some(false), Some(false), Some(true)]);
    }

    #[test]
    fn decoder_138_selects_one_line_low_and_goes_all_high_when_disabled() {
        // a0=1 a1=0 a2=1 -> sel=5; enabled.
        let mut levels = [false; 14];
        levels[0] = true; // a0
        levels[2] = true; // a2
        levels[5] = true; // e3 HIGH (active)
        // e1n, e2n already LOW (active).
        let out = evaluate(kinds::IC_74HC138, &levels);
        let expected: Vec<Option<bool>> = (0..8).map(|i| Some(i != 5)).collect();
        assert_eq!(out, expected);

        levels[5] = false; // e3 LOW -> disabled
        let disabled = evaluate(kinds::IC_74HC138, &levels);
        assert_eq!(disabled, vec![Some(true); 8]);
    }

    #[test]
    fn bcd47_decodes_zero_and_lamp_test_overrides_everything() {
        // bcd = 0, every control inactive (HIGH): "0" glyph, electrical.
        let mut levels = [false; 14];
        levels[2] = true; // lt_n inactive
        levels[3] = true; // rbo_n inactive
        levels[4] = true; // rbi_n inactive
        let zero = evaluate(kinds::IC_74LS47, &levels);
        // segs[0] = [T,T,T,T,T,T,F] (a..g); electrical order e,d,c,b,a,g,f.
        assert_eq!(zero, vec![Some(false), Some(false), Some(false), Some(false), Some(false), Some(true), Some(false)]);

        levels[2] = false; // lt_n LOW -> lamp test, every segment sinks (electrical LOW)
        let lamp_test = evaluate(kinds::IC_74LS47, &levels);
        assert_eq!(lamp_test, vec![Some(false); 7]);
    }

    #[test]
    fn transceiver_245_drives_only_the_selected_side_and_mutes_on_oe() {
        // dir=1 (A->B): a1 HIGH -> b1 HIGH, a1 itself undriven.
        let mut levels = [false; 18];
        levels[0] = true; // dir
        levels[2] = true; // a1
        let a_to_b = evaluate(kinds::IC_74LS245, &levels);
        assert_eq!(a_to_b[0], None, "a1 undriven while it is the source side");
        assert_eq!(a_to_b[1], Some(true), "b1 = a1");
        for g in 1..8 {
            assert_eq!(a_to_b[g * 2], None, "a_i undriven (source side) for gate {g}");
            assert_eq!(a_to_b[g * 2 + 1], Some(false), "b_i = a_i = LOW for gate {g}");
        }

        levels[1] = true; // /oe HIGH -> nothing driven, either side
        let disabled = evaluate(kinds::IC_74LS245, &levels);
        assert!(disabled.iter().all(|v| v.is_none()));
    }

    // ---- the golden table: every row replayed through evaluate() ----------

    #[test]
    fn golden_table_replays_classics_own_evaluator_exactly() {
        assert!(!combinational_golden::GOLDEN_ROWS.is_empty(), "the generator must have run at least once");
        for &(kind, input_bits, present_mask, level_mask) in combinational_golden::GOLDEN_ROWS {
            let l = logic_pin_count(kind);
            assert!(l > 0, "golden row names kind {kind}, which this module does not own");
            let levels: Vec<bool> = (0..l).map(|i| (input_bits >> i) & 1 == 1).collect();
            let result = evaluate(kind, &levels);
            let out_len = output_capable(kind).len();
            assert_eq!(result.len(), out_len, "kind {kind}, input_bits {input_bits:#x}");
            for (i, got) in result.iter().enumerate() {
                let present = (present_mask >> i) & 1 == 1;
                let level = (level_mask >> i) & 1 == 1;
                match got {
                    Some(v) => {
                        assert!(present, "kind {kind}, input_bits {input_bits:#x}: slot {i} is Some but golden says absent");
                        assert_eq!(*v, level, "kind {kind}, input_bits {input_bits:#x}: slot {i} level mismatch");
                    }
                    None => {
                        assert!(!present, "kind {kind}, input_bits {input_bits:#x}: slot {i} is None but golden says present");
                    }
                }
            }
        }
    }
}
