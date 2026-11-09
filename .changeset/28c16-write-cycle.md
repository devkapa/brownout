---
"brownout": patch
---

A 28C16 EEPROM now finishes a write in 0.5 ms, the AT28C16 datasheet's typical write-cycle time, instead of 10 ms. Both EEPROM kinds shared one hardcoded 10 ms, which is the AT28C256's maximum, so a 28C16 sat busy about twenty times longer than the real part, and a program that polls or waits for the write to end waited that long too. The 28C256 keeps 10 ms. The write time is now one per-kind table (`EEPROM_T_WC`) that the engine and the WASM core compiler both read, so the two engines cannot drift apart on it.
