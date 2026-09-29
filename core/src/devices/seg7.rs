//! 7-segment displays, `seg7_cc` and `seg7_ca` (core/README.md, "Kernel
//! semantics: LEDs and displays (W1e)"). Owned by W1e worker E2a. Pure
//! functions, reached through [`super::led::junctions`] and
//! [`super::led::readout`].
//!
//! The public API below is the orchestrator's lock, declared before dispatch
//! so no worker needs to touch `devices/mod.rs`. The bodies are stubs until
//! E2a lands (the underscores go with them): nothing calls them yet.

use super::led::Junction;
use crate::program::Device;

/// The stamped common node (LEDs point 14): `com` unless `OPEN`, else `com2`
/// unless `OPEN`, else none, which is classic's order.
pub fn common_node(_device: &Device) -> Option<usize> {
    todo!("W1e E2a")
}

/// One junction per segment that is not `OPEN`, when there is a common node;
/// segment anode on `seg7_cc`, cathode on `seg7_ca`.
pub fn junctions(_device_index: usize, _device: &Device, _ambient_c: f64) -> Vec<Junction> {
    todo!("W1e E2a")
}

/// The sum of the segments' positive currents and eight channel values. No
/// `failed` flag: `seg7` has no failure surface (Q-O22).
pub fn readout(_device: &Device, _junctions: &[Junction], _voltages: &[f64], _element_i: &mut f64, _channels: &mut [f64]) {
    todo!("W1e E2a")
}
