---
"brownout": patch
---

On the Rust/WASM core, a `bench_psu` in current limit whose rail has a capacitor on it and an esr-0 capacitor loop through the rail into a part the core steps, beside a part it solves in closed form, could read badly wrong: capacitor currents of about 12 A were published on a supply limited to 0.097 A in 0.7.0, and on another board, millions of amps. 3 of 4,200 random bench-supply boards have that structure, and no fixture, example or catalog board in the repository does. Such a board now declines at load with the reason `bench-psu-cc-rail-branches`, and `"auto"` runs it on the classic engine, which on these three boards reads far closer to the core's exact rail mode than the core's default did. Classic's trapezoid option reads some currents wrong on two of them, as it always did. The core's exact rail mode (`noRailShortcut`) is not affected. Some other current-limit boards the core reads wrong are not caught by this guard and wait for the core's own fix.
