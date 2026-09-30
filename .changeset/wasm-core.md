---
"brownout": major
---

brownout 1.0.0 ships the Rust/WASM simulation core behind the engine flag. The notes below are for code that drives the core directly; the ABI detail behind each one lives in `core/README.md`.

- The core's `noRailShortcut` exactness option is now chosen when a circuit is loaded and cannot be changed afterwards. It decides how the circuit around a battery or current-limited supply is partitioned, which is fixed at load. Setting it on an engine that already holds a circuit is rejected as an invalid argument. To run a circuit in exact rail mode, pass `noRailShortcut` in the load options, as the SAP-1 acceptance script and the compiler already do.
