---
"brownout": patch
---

On the Rust/WASM core, a circuit could freeze the simulation: a `bench_psu` in current limit charging a capacitor on its rail, with a load that draws less current as the rail rises (in the case found, an `ne555` whose output is wired to a negative source), sent a crossing search inside one simulated instant into a search that never ended, so `advance` never returned and the worker running it stopped responding. It came with 0.7.0 and was found on 1 of about 3,900 random bench-supply circuits. Fixed: such a circuit runs, and reads as it did before 0.7.0 and as the core's exact rail mode does. A crossing search that cannot finish now fails the advance instead (code `-4`, the message starting `a crossing search ran out of work`), which `"auto"` handles as any failed solve: the run goes on on classic, with `mid-run:solve-failed:...` as the reason.
