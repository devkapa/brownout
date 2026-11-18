---
"brownout": minor
---

Eight more part kinds run on the Rust/WASM core and are cleared for automatic use (`CORE_SUPPORTED_KINDS`): `potentiometer`, `trimmer`, `resistor_array`, `ldr`, `ferrite_bead`, `push_dpdt`, `current_source` and `signal_gen`. A circuit made of these and the kinds the core already ran goes to the core with no change on the host's side; every other circuit still runs on the classic engine. Each was checked on both engines against a repository fixture (the per-kind table reads "pass"), its snapshot, rollback and carried-reload state held by tests, and its gate-1 cases run on the core. A potentiometer's knob, an LDR's lux, a DPDT's position and a current source's current are live edits; a generator's waveform, frequency and `enabled` are load-time parameters, so editing one reloads the circuit and carries every capacitor and latch across, exactly as classic reloads it. The 38 kinds the core already ran read bit for bit as before.

Two recorded divergences from classic: a ferrite bead (and a rated resistor or potentiometer) that reaches its overload limit latches at the exact crossing on the core, where classic rounds the latch to the end of its step; and a `resistor_array` whose channels are fed by floating supplies of their own declines to the classic engine, as any floating ideal source does.
