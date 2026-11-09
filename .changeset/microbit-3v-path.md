---
"brownout": patch
---

A powered micro:bit with a supply on its 3V pad now converges. Its pad was an ideal 3.3 V source, so a bench PSU there made the system singular, the same bug the Uno had. The V1 board reaches the pad through D1, an Infineon BAT60A Schottky, from the interface chip's 3.3 V USB regulator (micro:bit V1.3B and V1.5 schematics, "Power Supply" sheet). The engine now models that path, so a supply on the pad above the regulator's output reverse-biases D1.

`supply_overvoltage` now also covers the micro:bit: its 3V pad is the nRF51822's VDD, absolute maximum 3.9 V (nRF51822 Product Specification v2.0, Table 16).

Readings that change on a powered micro:bit (`power` on):

- 3V pad, idle (15 mA): 3.300 V to 3.166 V, the BAT60A drop.
- Everything referenced to the pad moves with it: P0-P2 HIGH levels and the current they drive (about -4%), and the board's supply current, 15.0 mA to 14.4 mA at idle.
- A micro:bit with `power` off is unchanged.
