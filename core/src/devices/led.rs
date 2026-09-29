//! LED junctions (core/README.md, "Kernel semantics: LEDs and displays
//! (W1e)"). Owned by W1e worker E2a. Pure functions: no solver state.
//!
//! The public API below is the orchestrator's lock, declared before dispatch
//! so no worker needs to touch `devices/mod.rs`. The bodies are stubs until
//! E2a lands (the underscores go with them): nothing in the solver calls
//! them yet. `junctions` and `readout` cover all five kinds and hand codes 33
//! and 34 to [`super::seg7`].

use crate::program::Device;

/// One junction's law at one temperature (LEDs point 2). Takes `t_c` so W1g
/// can feed a held junction temperature without a signature change.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JunctionLaw {
    pub vtn: f64,
    pub is: f64,
    pub vsat: f64,
}

impl JunctionLaw {
    /// `is_override` is a `led`'s `Is` param when `isSet = 1`.
    pub fn at(_vf25: f64, _i_rated: f64, _n: f64, _is_override: Option<f64>, _t_c: f64) -> JunctionLaw {
        todo!("W1e E2a")
    }

    /// `I(v)`, anode to cathode, GMIN excluded.
    pub fn current(&self, _v: f64) -> f64 {
        todo!("W1e E2a")
    }

    /// `(g, ieq)` about `v_prev`: `g` includes `GMIN_D`, `ieq` does not.
    pub fn companion(&self, _v_prev: f64) -> (f64, f64) {
        todo!("W1e E2a")
    }
}

/// One junction of one device (LEDs point 1). Listed even when self-shorted,
/// so the device's channel indexing stays fixed; such a junction stamps
/// nothing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Junction {
    /// The owning device's record index.
    pub device: usize,
    /// The device's channel for this junction (0 for a `led`, which has no
    /// channel slots).
    pub channel: usize,
    pub anode: usize,
    pub cathode: usize,
    pub law: JunctionLaw,
}

impl Junction {
    /// LEDs point 1: a junction whose two pins share a node stamps nothing
    /// and does not classify.
    pub fn stamped(&self) -> bool {
        self.anode != self.cathode
    }
}

/// The junctions of one LED-family device, fixed at load, at `ambient_c`
/// (LEDs points 1 and 3). Empty for a `burnt` device.
pub fn junctions(_device_index: usize, _device: &Device, _ambient_c: f64) -> Vec<Junction> {
    todo!("W1e E2a")
}

/// The device's `elementI` and channel values from the published voltages
/// ("Currents"; LEDs point 13). `failed` is W1g's `led_failed`, always
/// `false` in W1e; it takes the same path as `burnt`.
pub fn readout(_device: &Device, _junctions: &[Junction], _voltages: &[f64], _failed: bool, _element_i: &mut f64, _channels: &mut [f64]) {
    todo!("W1e E2a")
}
