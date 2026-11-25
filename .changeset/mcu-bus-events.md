---
"brownout": minor
---

Arduino boards report their serial, I2C and SPI traffic for protocol decoders. `SimEngine.drainMcuBusEvents()`
returns each board's events since the last call, keyed by component id, each with its start time `t` in seconds of
simulation: UART frames sent and received (with the byte and the frame's length in cycles), I2C starts, restarts,
stops, address bytes and data bytes (with who acknowledged), and SPI exchanges (the byte out and the byte in). The
events come from the peripherals, which know each byte exactly, where sampling the pins would miss most bits. A board
keeps the newest 4096 undrained events and counts the rest (`ArduinoMcu.drainBusEvents().dropped`).
