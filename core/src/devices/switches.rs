//! `switch`, `push_button`, `spdt_switch` and `dip_switch` (core/README.md's
//! kind-code table; spec §7.6). None of these kinds has a failure hook, a
//! rating or contact bounce (spec §7.6's opening line): a closed contact is
//! a literal 1 mΩ, exactly as classic hard-codes it (`passives.ts:462` &c);
//! an open one stamps nothing.
//!
//! Every function here is stateless: the caller (`solve/static_part.rs`)
//! owns the mutable, CURRENT `params` a live-parameter change
//! (`bo_engine_set_params`, owner decision 1) writes into, and calls
//! `stamp`/`current` fresh on every resolve — unlike the resistor, a
//! switch's electrical state can change after load, so nothing here is ever
//! cached the way a resistor's stamp is baked into the partition's pristine
//! base.

use crate::devices::Contribution;
use crate::kinds;

/// A closed contact's conductance: `1 / 0.001 Ω` (README's "Currents" note
/// and spec §7.1's literal switch-contact constant, shared by every kind in
/// this module).
const CLOSED_CONDUCTANCE: f64 = 1.0 / 0.001;

/// `Boolean(params.closed)` (classic's own rule, mirrored — spec §7.6): a
/// live param's value is already restricted to `0.0`/`1.0` by
/// `kinds::is_switch_value`, so this is just "nonzero".
fn is_closed(v: f64) -> bool {
    v != 0.0
}

/// Every pin-index pair (into the device's own `pins`) this kind can ever
/// carry a contact between — regardless of the device's CURRENT parameter
/// state. Used only for partition topology (`partition::build` unions each
/// pair unconditionally, exactly like a resistor); never for the actual
/// electrical stamp below, which is what decides whether a given pair is
/// closed right now. Keeping the two separate is what makes a live toggle
/// never move a node from one partition to another — see `partition.rs`'s
/// module doc comment.
///
/// Pin order, per core/README.md's record-layout table:
/// - `switch`/`push_button`: `pins[0]`, `pins[1]` (positional) — one contact.
/// - `spdt_switch`: `com` (0), `b` (1), `c` (2) (by id) — two contacts,
///   `com`-`b` and `com`-`c`.
/// - `dip_switch`: `a1, b1, ..., ak, bk` (by id) — one contact per position,
///   `pin_count / 2` of them (`kinds::dip_switch_check` already guarantees
///   `pin_count == 2 * k`).
pub fn contact_pairs(kind: u32, pin_count: usize) -> Vec<(usize, usize)> {
    match kind {
        kinds::SWITCH | kinds::PUSH_BUTTON => vec![(0, 1)],
        kinds::SPDT_SWITCH => vec![(0, 1), (0, 2)],
        kinds::DIP_SWITCH => (0..pin_count / 2).map(|i| (2 * i, 2 * i + 1)).collect(),
        _ => Vec::new(),
    }
}

/// Whether contact `contact_index` (as `contact_pairs` orders them) is
/// closed right now, given this kind's current `params`
/// (core/README.md's record-layout table):
/// - `switch`/`push_button`: `[closed]` — the one contact.
/// - `spdt_switch`: `[sel]` — `sel == 0` selects `b` (contact 0), anything
///   else selects `c` (contact 1). `kinds::single_switch_check` already
///   restricts a live `sel` to `0.0`/`1.0`, so this is a plain equality, not
///   classic's raw `Number(position) === 0` (which also treats `NaN` as
///   "not 0"); the two agree on every value this program format can ever
///   carry.
/// - `dip_switch`: `[k, sw1..sw8]` — contact `i` (0-based) is closed when
///   `sw_(i+1)` (`params[i + 1]`) is nonzero. Only the first `k` contacts
///   exist as pins at all; `contact_pairs` already stops there.
fn is_contact_closed(kind: u32, contact_index: usize, params: &[f64]) -> bool {
    match kind {
        kinds::SWITCH | kinds::PUSH_BUTTON => is_closed(params[0]),
        kinds::SPDT_SWITCH => {
            let selects_c = params[0] != 0.0;
            if contact_index == 0 {
                !selects_c
            } else {
                selects_c
            }
        }
        kinds::DIP_SWITCH => is_closed(params[contact_index + 1]),
        _ => false,
    }
}

/// This device's CURRENT stamp: one [`Contribution`] per contact
/// `contact_pairs` names, conductance [`CLOSED_CONDUCTANCE`] when that
/// specific contact is closed right now and `0.0` (README: "open stamps
/// nothing" — a zero conductance term contributes nothing to the matrix
/// build in `solve/static_part.rs`, which IS "no stamp") when it is not.
pub fn stamp(kind: u32, pin_count: usize, params: &[f64]) -> Vec<Contribution> {
    contact_pairs(kind, pin_count)
        .into_iter()
        .enumerate()
        .map(|(contact_index, (pin_a, pin_b))| {
            let g = if is_contact_closed(kind, contact_index, params) { CLOSED_CONDUCTANCE } else { 0.0 };
            Contribution::conductance(pin_a, pin_b, g)
        })
        .collect()
}

/// This device's published current (README's "Currents" note): the sum of
/// every contact's own conductance times its own terminal voltage
/// difference.
///
/// - `switch`/`push_button`: exactly one contact, so this is just its
///   current (`0` when open).
/// - `spdt_switch`: exactly one of the two contacts ever has a nonzero
///   conductance at a time, so the "open" contact's term is always `0` and
///   this sum IS "the current through the contact in use" — the Q-O26 fix
///   (README: "This is the Q-O26 fix, landed in classic first; classic used
///   to report `com` to `b` at every position") — with no special-casing
///   needed here: it falls out of summing a zero times anything, rather
///   than classic's old bug of always reading `com`-`b` regardless of
///   position.
/// - `dip_switch`: literally "the sum over the closed positions' contact
///   currents" the README asks for.
///
/// `pin_voltages` is parallel to the device's own `pins` (already resolved
/// to whichever partition or fixed value each pin currently holds).
pub fn current(kind: u32, pin_count: usize, params: &[f64], pin_voltages: &[f64]) -> f64 {
    stamp(kind, pin_count, params).into_iter().map(|c| c.conductance * (pin_voltages[c.pin_a] - pin_voltages[c.pin_b])).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    const CLOSED: f64 = 1.0;
    const OPEN: f64 = 0.0;

    // ---- switch / push_button ------------------------------------------

    #[test]
    fn switch_closed_stamps_the_literal_1_milliohm_contact() {
        let s = stamp(kinds::SWITCH, 2, &[CLOSED]);
        assert_eq!(s, vec![Contribution::conductance(0, 1, 1000.0)]);
    }

    #[test]
    fn switch_open_stamps_zero_conductance() {
        let s = stamp(kinds::SWITCH, 2, &[OPEN]);
        assert_eq!(s, vec![Contribution::conductance(0, 1, 0.0)]);
    }

    #[test]
    fn push_button_behaves_exactly_like_switch() {
        assert_eq!(stamp(kinds::PUSH_BUTTON, 2, &[CLOSED]), stamp(kinds::SWITCH, 2, &[CLOSED]));
        assert_eq!(stamp(kinds::PUSH_BUTTON, 2, &[OPEN]), stamp(kinds::SWITCH, 2, &[OPEN]));
    }

    #[test]
    fn switch_current_is_zero_when_open_and_ohms_law_when_closed() {
        // 10 V across the contact.
        assert_eq!(current(kinds::SWITCH, 2, &[OPEN], &[10.0, 0.0]), 0.0);
        let i = current(kinds::SWITCH, 2, &[CLOSED], &[10.0, 0.0]);
        assert!((i - 10.0 * 1000.0).abs() < 1e-9, "got {i}");
    }

    // ---- spdt_switch: com(0), b(1), c(2) --------------------------------

    #[test]
    fn spdt_position_0_selects_b_and_opens_c() {
        let s = stamp(kinds::SPDT_SWITCH, 3, &[0.0]);
        assert_eq!(s, vec![Contribution::conductance(0, 1, 1000.0), Contribution::conductance(0, 2, 0.0)]);
    }

    #[test]
    fn spdt_position_1_selects_c_and_opens_b() {
        let s = stamp(kinds::SPDT_SWITCH, 3, &[1.0]);
        assert_eq!(s, vec![Contribution::conductance(0, 1, 0.0), Contribution::conductance(0, 2, 1000.0)]);
    }

    #[test]
    fn spdt_current_reports_the_contact_in_use_not_always_com_to_b() {
        // com = 5V, b = 0V, c = 5V: with com-b (1000 S) and com-c open, all
        // the current is on com-b. Selecting c instead must report the
        // com-c current, not `(V(com)-V(b))/0.001` regardless of position
        // (the Q-O26 bug this module's doc comment calls out).
        let pin_voltages = [5.0, 0.0, 5.0];
        let i_b_selected = current(kinds::SPDT_SWITCH, 3, &[0.0], &pin_voltages);
        assert!((i_b_selected - 5000.0).abs() < 1e-6, "got {i_b_selected}");

        let i_c_selected = current(kinds::SPDT_SWITCH, 3, &[1.0], &pin_voltages);
        // com and c are both at 5V: zero volts across the selected contact,
        // so the reported current must be ~0, not the nonsensical "5000 A"
        // classic's bug produced by always reading com-b.
        assert!(i_c_selected.abs() < 1e-6, "got {i_c_selected}, expected ~0 (com-c has no voltage across it)");
    }

    #[test]
    fn spdt_any_nonzero_position_selects_c_nan_included() {
        // README: "Number(params.position) === 0 ? 0 : 1 (0 selects b;
        // anything else, NaN included, selects c)" describes the COMPILER's
        // rule that produces the program's `sel` value; by the time it
        // reaches this module (and `kinds::single_switch_check`), `sel` is
        // already restricted to 0.0/1.0. This test only pins that any
        // nonzero `sel` this module actually receives selects c.
        let s = stamp(kinds::SPDT_SWITCH, 3, &[1.0]);
        assert_eq!(s[1].conductance, 1000.0);
    }

    // ---- dip_switch: a1,b1,...,ak,bk -------------------------------------

    #[test]
    fn dip_switch_k_2_stamps_two_independent_contacts() {
        // params = [k, sw1, sw2, sw3..sw8]; k=2, sw1 closed, sw2 open.
        let mut params = vec![2.0, 1.0, 0.0];
        params.extend([0.0; 6]); // sw3..sw8, inert past k=2.
        let s = stamp(kinds::DIP_SWITCH, 4, &params);
        assert_eq!(s, vec![Contribution::conductance(0, 1, 1000.0), Contribution::conductance(2, 3, 0.0)]);
    }

    #[test]
    fn dip_switch_k_8_stamps_eight_independent_contacts_all_closed() {
        let mut params = vec![8.0];
        params.extend([1.0; 8]); // sw1..sw8, all closed.
        let s = stamp(kinds::DIP_SWITCH, 16, &params);
        assert_eq!(s.len(), 8);
        for (i, c) in s.iter().enumerate() {
            assert_eq!(*c, Contribution::conductance(2 * i, 2 * i + 1, 1000.0), "contact {i}");
        }
    }

    #[test]
    fn dip_switch_current_is_the_sum_over_closed_positions_only() {
        // k=2: position 1 closed with 3V across it (0.003 A), position 2
        // open with 7V across it (must not contribute).
        let params = [2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let pin_voltages = [3.0, 0.0, 7.0, 0.0];
        let i = current(kinds::DIP_SWITCH, 4, &params, &pin_voltages);
        assert!((i - 3000.0).abs() < 1e-6, "got {i}");
    }

    #[test]
    fn dip_switch_extra_sw_params_past_k_are_inert() {
        // k=2, only 4 pins exist, so contact_pairs must stop at 2 contacts
        // even though sw3..sw8 carry nonzero (closed) values.
        let mut params = vec![2.0, 0.0, 0.0];
        params.extend([1.0; 6]); // sw3..sw8 "closed", but no pins 5..16 exist.
        let s = stamp(kinds::DIP_SWITCH, 4, &params);
        assert_eq!(s.len(), 2, "only k=2 contacts may be stamped");
    }

    #[test]
    fn contact_pairs_match_pin_count_for_every_dip_size() {
        for k in 2..=8usize {
            let pairs = contact_pairs(kinds::DIP_SWITCH, 2 * k);
            assert_eq!(pairs.len(), k);
            for (i, (a, b)) in pairs.iter().enumerate() {
                assert_eq!(*a, 2 * i);
                assert_eq!(*b, 2 * i + 1);
            }
        }
    }
}
