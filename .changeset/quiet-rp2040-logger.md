---
"brownout": patch
---

`RP2040Mcu` no longer floods the console. rp2040js defaults to a Debug-level `ConsoleLogger`, so every USB transfer, SEV and unimplemented peripheral access was printed: a Pico booting MicroPython logged 2,000+ lines in its first seconds, and in de:volt the worker fell behind real time and the page stopped responding. The core now logs errors only, set on every boot including `reset()`. Errors still print and throw as before. Arduino (avr8js) never logged, so it is unchanged.
