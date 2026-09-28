---
"brownout": patch
---

`RP2040Mcu` now holds the on-die temperature sensor (ADC4) at 27 °C. The engine never fed that channel and rp2040js starts it at 0 counts, so MicroPython's `ADC(4).read_u16()` returned 0 and the datasheet formula `27 - (V - 0.706) / 0.001721` gave 437 °C. Each boot, including `reset()`, now seeds 876 counts (the datasheet's 0.706 V at 27 °C against 3.3 V), which reads back as 27.04 °C.
