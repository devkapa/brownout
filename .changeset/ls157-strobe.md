---
"brownout": patch
---

The 74LS157's `/OE` strobe now forces every `Y` output LOW, instead of releasing the bus, matching the real SN74LS157: its `/G` pin is a strobe, not a tri-state enable, and TI's function table (SDLS058) shows `/G` HIGH driving every output LOW regardless of SELECT or the data inputs. The tri-state behaviour this pin used to model belongs to a different part, the 74LS257. A strobed 157's four `digitalState` keys (`1y`..`4y`) now stay present at 0 instead of disappearing, and an open `/oe` — which reads HIGH in most 10 ms floating buckets — now holds every `Y` LOW instead of releasing the bus to whatever pulls it. The 74LS245's `/OE` is unchanged: it still tri-states both the `a` and `b` sides.
