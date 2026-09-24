//! `brownout-core`: the W0 kernel, compiled to WebAssembly. See
//! `core/README.md` for the contract (program format v1, C ABI v1, W0
//! kernel semantics) — this crate implements exactly that.
//!
//! Module layout mirrors the contract's division of labour:
//! - [`program`]: the program format v1 reader. Turns a `&[f64]` into a
//!   validated [`program::Program`], or a precise [`error::BoError`] —
//!   never a panic.
//! - [`lu`]: dense LU with partial pivoting, LINPACK layout.
//! - [`engine`]: the W0 kernel itself — fixed-node derivation, union-find
//!   partitioning, the per-partition solve, and the KCL current pass. A
//!   plain safe Rust API with no `extern "C"` on it; `abi` is the only
//!   caller.
//! - [`abi`]: the thin `extern "C"` shim (handle table, global last-error
//!   buffer, pointer marshalling). No physics lives here.

pub mod engine;
pub mod error;
pub mod lu;
pub mod program;

mod abi;
