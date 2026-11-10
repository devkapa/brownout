//! `brownout-core`: the partitioned event-driven circuit core, compiled to
//! WebAssembly. `core/README.md` is the contract (program format v2, C ABI
//! v2, kernel semantics); this crate implements it.
//!
//! Module layout (the W1 plan's orchestrator lock):
//! - [`kinds`]: kind codes and record shapes. Orchestrator-owned.
//! - [`program`]: the program reader, which returns a validated program or a
//!   precise error, never a panic.
//! - [`kernel`], [`events`], [`drivers`]: the event loop, the heap, and IC
//!   output drivers with inertial delays.
//! - [`engine`]: load orchestration.
//! - [`partition`]: fixed nodes, partitioning and partition classes.
//! - [`solve`]: partition solvers. [`lu`] is dense LU in LINPACK layout.
//! - [`devices`]: per-kind stamps and currents for the passive and switch
//!   kinds.
//! - [`readout`]: digital and channel slots and the harness trace.
//! - [`carry`]: W2c's load carry-forward records.
//! - [`snapshot`]: W2a's whole-engine snapshots behind opaque ids.
//! - [`abi`]: the thin `extern "C"` shim. No physics lives there.

pub mod carry;
pub mod devices;
pub mod digital;
pub mod drivers;
pub mod engine;
pub mod error;
pub mod events;
pub mod kernel;
pub mod kinds;
pub mod lu;
pub mod partition;
pub mod program;
pub mod rails;
pub mod readout;
pub mod shared;
pub mod slow;
pub mod snapshot;
pub mod solve;

mod abi;
