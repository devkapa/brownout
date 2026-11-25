---
"brownout": patch
---

On the Rust/WASM core's exact rail mode (`noRailShortcut`), a battery's own partition now steps TR-BDF2 like every other stepped partition, instead of backward Euler throughout. A 555 astable on a battery ran up to 1.5 % slow in exact mode, more the longer the host's steps (fixture 77: 1.40 % at 1 ms host steps against an independent reference, 0.10 % at 10 us); it now runs within 0.1 % at every host step. The cell's EMF and internal resistance are still taken at each step's start, and a step is shortened when the cell's held terminal voltage would move by more than half the tolerance within it (steeper near the end of discharge). The default rail mode was already within 0.02 % and is unchanged.
