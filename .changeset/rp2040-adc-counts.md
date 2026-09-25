---
"brownout": patch
---

`RP2040Mcu.setAnalogVolts` now converts the pin voltage to 12-bit ADC counts. rp2040js reads `adc.channelValues` as raw conversion results (0 to 4095 for 0 V to ADC_VREF), but the core wrote volts there, so 3.3 V on GP26 read as 3 counts and MicroPython's `ADC(Pin(26)).read_u16()` returned 48 instead of 65535. The voltage is now clamped to 0..`vcc` and scaled to `round(volts / vcc * 4095)`, with `vcc` the Pico's 3V3 rail that the engine already passes (on the Pico, ADC_VREF is that rail filtered). This covers ADC0 to ADC2 (GP26 to GP28). The temperature-sensor channel (ADC4) is not fed by the engine and still reads 0.
