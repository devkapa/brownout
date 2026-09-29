//! 7-segment displays, `seg7_cc` and `seg7_ca` (core/README.md, "Kernel
//! semantics: LEDs and displays (W1e)"). Owned by W1e worker E2a. Pure
//! functions, reached through [`super::led::junctions`] and
//! [`super::led::readout`].
//!
//! Pin order (record-layout table, "by id"): `a b c d e f g dp com com2`,
//! indices 0-9. `SEGMENTS` (0-7) covers `a`..`dp` in channel order; `COM`
//! (8) and `COM2` (9) are the two candidate common pins (LEDs point 14).

use super::led::{is_open, Junction, JunctionLaw, N_LED};
use crate::kinds::{self, seg7_param};
use crate::program::Device;

/// How many segment channels a `seg7` carries (`a`..`dp`), and the channel
/// index each shares with its own pin index (record-layout table: segment
/// pins are the device's first 8 pins, in the same `a..dp` order the channel
/// table uses).
const SEGMENTS: std::ops::Range<usize> = 0..8;
const COM: usize = 8;
const COM2: usize = 9;

/// The stamped common node (LEDs point 14): `com` unless `OPEN`, else `com2`
/// unless `OPEN`, else none, which is classic's order.
pub fn common_node(device: &Device) -> Option<usize> {
    if !is_open(device, COM) {
        Some(device.pins[COM].node)
    } else if !is_open(device, COM2) {
        Some(device.pins[COM2].node)
    } else {
        None
    }
}

/// One junction per segment that is not `OPEN`, when there is a common node;
/// segment anode on `seg7_cc`, cathode on `seg7_ca`. Empty when neither `com`
/// nor `com2` is wired (LEDs point 14: "the device stamps nothing").
pub fn junctions(device_index: usize, device: &Device, ambient_c: f64) -> Vec<Junction> {
    let Some(com_node) = common_node(device) else {
        return Vec::new();
    };
    let p = &device.params;
    // One law shared by every segment: classic derives Vf/Is/vSat once,
    // outside its per-segment loop (DI:1208-1214).
    let law = JunctionLaw::at(p[seg7_param::VF], p[seg7_param::I_RATED], N_LED, None, ambient_c);
    let common_anode = device.kind == kinds::SEG7_CA;
    SEGMENTS
        .filter(|&seg| !is_open(device, seg))
        .map(|seg| {
            let seg_node = device.pins[seg].node;
            let (anode, cathode) = if common_anode { (com_node, seg_node) } else { (seg_node, com_node) };
            Junction { device: device_index, channel: seg, anode, cathode, law }
        })
        .collect()
}

/// The sum of the segments' positive currents and eight channel values. No
/// `failed` flag: `seg7` has no failure surface (Q-O22).
pub fn readout(_device: &Device, junctions: &[Junction], voltages: &[f64], element_i: &mut f64, channels: &mut [f64]) {
    // Zero every channel first: an OPEN segment, or no common pin at all,
    // leaves no Junction for that channel (LEDs point 13: "0 for an OPEN
    // segment or when no common pin is stamped").
    channels.iter_mut().for_each(|c| *c = 0.0);
    let mut sum = 0.0;
    for j in junctions {
        let i = j.law.current(voltages[j.anode] - voltages[j.cathode]).max(0.0);
        if j.channel < channels.len() {
            channels[j.channel] = i;
        }
        sum += i;
    }
    *element_i = sum;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::program::{PinRecord, PIN_FLAG_OPEN};

    fn pin(node: usize) -> PinRecord {
        PinRecord { node, flags: 0, open_hash: 0, dead_hash: 0 }
    }

    fn open_pin(node: usize) -> PinRecord {
        PinRecord { node, flags: PIN_FLAG_OPEN, open_hash: 0, dead_hash: 0 }
    }

    /// `a b c d e f g dp com com2`, params `[vf, iRated]`.
    fn seg7_device(kind: u32, pins: Vec<PinRecord>) -> Device {
        assert_eq!(pins.len(), 10);
        Device { kind, component_index: 0, pins, params: vec![2.0, 0.02], blob: vec![0.0] }
    }

    fn all_wired_pins(com_node: usize) -> Vec<PinRecord> {
        // Segments on nodes 1..8, com on `com_node`, com2 left open.
        let mut pins: Vec<PinRecord> = (1..=8).map(pin).collect();
        pins.push(pin(com_node)); // com
        pins.push(open_pin(0)); // com2
        pins
    }

    #[test]
    fn common_node_prefers_com_over_com2() {
        let mut pins = all_wired_pins(9);
        pins[COM2] = pin(20); // com2 also wired, but com must win
        let d = seg7_device(kinds::SEG7_CC, pins);
        assert_eq!(common_node(&d), Some(9));
    }

    #[test]
    fn common_node_falls_back_to_com2_when_com_is_open() {
        let mut pins = all_wired_pins(9);
        pins[COM] = open_pin(0);
        pins[COM2] = pin(20);
        let d = seg7_device(kinds::SEG7_CC, pins);
        assert_eq!(common_node(&d), Some(20));
    }

    #[test]
    fn common_node_is_none_when_both_are_open() {
        let mut pins = all_wired_pins(9);
        pins[COM] = open_pin(0);
        let d = seg7_device(kinds::SEG7_CC, pins); // com2 already open from all_wired_pins
        assert_eq!(common_node(&d), None);
    }

    #[test]
    fn no_common_pin_means_no_junctions_at_all() {
        let mut pins = all_wired_pins(9);
        pins[COM] = open_pin(0);
        let d = seg7_device(kinds::SEG7_CC, pins);
        assert!(junctions(0, &d, 25.0).is_empty());
    }

    #[test]
    fn open_segment_has_no_junction() {
        let mut pins = all_wired_pins(9);
        pins[3] = open_pin(0); // segment 'd' open
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        assert_eq!(js.len(), 7, "one junction per non-OPEN segment");
        assert!(js.iter().all(|j| j.channel != 3), "no junction listed for the OPEN segment");
    }

    #[test]
    fn seg7_cc_anode_is_the_segment_cathode_is_common() {
        let pins = all_wired_pins(9);
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        assert_eq!(js.len(), 8);
        for j in &js {
            assert_eq!(j.anode, j.channel + 1, "segment node is anode on seg7_cc");
            assert_eq!(j.cathode, 9, "common is cathode on seg7_cc");
        }
    }

    #[test]
    fn seg7_ca_polarity_is_reversed() {
        let pins = all_wired_pins(9);
        let d = seg7_device(kinds::SEG7_CA, pins);
        let js = junctions(0, &d, 25.0);
        assert_eq!(js.len(), 8);
        for j in &js {
            assert_eq!(j.anode, 9, "common is anode on seg7_ca");
            assert_eq!(j.cathode, j.channel + 1, "segment node is cathode on seg7_ca");
        }
    }

    #[test]
    fn dp_is_a_segment_like_the_others() {
        let pins = all_wired_pins(9);
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        assert!(js.iter().any(|j| j.channel == 7), "dp (channel 7) must be present");
    }

    #[test]
    fn self_shorted_segment_is_listed_but_unstamped() {
        // Segment 'a' wired to the same node as com: listed (not OPEN), but
        // its junction stamps nothing.
        let mut pins = all_wired_pins(9);
        pins[0] = pin(9); // segment a on the common's own node
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        let a = js.iter().find(|j| j.channel == 0).expect("segment a still listed");
        assert!(!a.stamped());
        assert_eq!(a.law.current(0.0), 0.0);
    }

    #[test]
    fn com_and_com2_open_flags_read_with_the_shared_helper() {
        // Sanity: seg7's is_open reuse of led::is_open reads the same
        // PIN_FLAG_OPEN bit `program.rs` defines.
        let pins = all_wired_pins(9);
        let d = seg7_device(kinds::SEG7_CC, pins);
        assert!(!is_open(&d, COM));
        assert!(is_open(&d, COM2));
    }

    // ---- readout -------------------------------------------------------

    #[test]
    fn readout_sums_positive_parts_in_channel_order() {
        let pins = all_wired_pins(9); // segments on nodes 1..8, com on node 9
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        // 10 nodes: 0 (unused/ground-ish), 1..8 segments, 9 common. Drive
        // segments a..d forward (2.2V), e..dp reverse (-1V), common at 0V.
        let mut voltages = [0.0; 10];
        for seg in 0..4 {
            voltages[seg + 1] = 2.2;
        }
        for seg in 4..8 {
            voltages[seg + 1] = -1.0;
        }
        voltages[9] = 0.0;
        let mut element_i = 0.0;
        let mut channels = [0.0; 8];
        readout(&d, &js, &voltages, &mut element_i, &mut channels);
        for seg in 0..4 {
            assert!(channels[seg] > 0.0, "segment {seg} should be lit");
        }
        for seg in 4..8 {
            assert_eq!(channels[seg], 0.0, "reverse-biased segment {seg} floors at 0");
        }
        let expected_sum: f64 = channels.iter().sum();
        assert!((element_i - expected_sum).abs() < 1e-15);
    }

    #[test]
    fn readout_with_no_common_pin_is_all_zero() {
        let mut pins = all_wired_pins(9);
        pins[COM] = open_pin(0);
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        assert!(js.is_empty());
        let voltages = [5.0; 10];
        let mut element_i = 999.0;
        let mut channels = [999.0; 8];
        readout(&d, &js, &voltages, &mut element_i, &mut channels);
        assert_eq!(element_i, 0.0);
        assert_eq!(channels, [0.0; 8]);
    }

    #[test]
    fn readout_zeroes_an_open_segments_channel() {
        let mut pins = all_wired_pins(9);
        pins[5] = open_pin(0); // segment 'f' open
        let d = seg7_device(kinds::SEG7_CC, pins);
        let js = junctions(0, &d, 25.0);
        let mut voltages = [3.0; 10];
        voltages[9] = 0.0; // common
        let mut element_i = 0.0;
        let mut channels = [999.0; 8];
        readout(&d, &js, &voltages, &mut element_i, &mut channels);
        assert_eq!(channels[5], 0.0);
        assert!(channels[0] > 0.0, "a non-open lit segment should still read forward current");
    }
}
