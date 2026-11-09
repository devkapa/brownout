---
"brownout": patch
---

New failure kind `supply_overvoltage`. An Uno or Nano whose 5V pin rises past the ATmega328P's 6.0 V absolute maximum (VCC), or a Pico whose 3V3 pin rises past the RP2040's 3.63 V (IOVDD, datasheet 5.5.3.1), now latches it on the board, with the pin, the reading and the limit. A typical cause is a bench PSU back-feeding a board through its supply pin. It changes no stamp: the board keeps running. Like other latched failures it holds after the supply comes back down, until `resetFailures()`. `SimFailureKind` gains the new member, so a host with an exhaustive map over failure kinds needs an entry for it.

In the operating-point report, a board with `supply_overvoltage` is marked `overRated: true`, not `failed`. The CSV gains a trailing `over_rated` column.
