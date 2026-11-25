---
"brownout": patch
---

On the Rust/WASM core, a board with catalog LEDs solves less. An LED whose junction temperature moves by 0.01 C used to empty the solution cache of every partition it sits in; the cache now keys on the LED's law, so a solve stored at one temperature is replayed when the LED returns to it, and the catalog SAP-1 spends 5 % fewer instructions on a battery supply and 16 % fewer on an ideal one. Readings can differ from 0.7.0 on such a board, inside the cache's own stated bounds: a replayed solve is within 1 uV of a fresh one, and a hub partition's within W0 across its curvature band. Boards with no thermal records read bit for bit as before.
