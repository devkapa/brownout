---
"brownout": patch
---

On the Rust/WASM core, a part that fails, trips or recovers, or is restored by `resetFailures`, while it sits straight across a battery's or bench supply's rail now has the chips it powers read at that instant. Before, the rail moved but a logic chip it powered on could show its old output for tens of milliseconds, until the rail drifted another 20 mV: on a 74LS00 whose rated load across a small cell burnt open, gate 0 stayed high 47 ms where it should have gone low at once, and a second part could fail during that time that never fails otherwise. The same holds for a switch opened or closed straight across the rail. Exact mode (`noRailShortcut`) already read the chip at once, and the default build now follows its order: the re-solve, then every chip it moved, then their outputs, all in the same instant. Circuits with no such part read bit for bit as before.
