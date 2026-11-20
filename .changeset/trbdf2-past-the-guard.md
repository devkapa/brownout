---
"brownout": patch
---

On the Rust/WASM core, a linear stepped partition whose fastest mode is stiff against the step now reads a capacitor's current within the tolerance of the closed form at every host stop. Past the extrapolation's stability guard a step used to be the plain trapezoid, whose error on a two-pole RC ladder piled up to 4.5 to 5.8 times the tolerance in the second capacitor's current at host stops of 1 ms or less; it is now the Richardson extrapolation of TR-BDF2 (third order, and its amplification stays below 1 for every real mode), at most 0.16 of the tolerance on that ladder. A step at or under the guard, and every board with no such partition, reads bit for bit as before; a step past the guard costs six solves in place of three.
