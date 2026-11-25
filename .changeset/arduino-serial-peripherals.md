---
"brownout": minor
---

Arduino boards now model the ATmega328P's USART, TWI (I2C), SPI and EEPROM. Sketches using Serial, Wire, SPI or
EEPROM no longer stall waiting on a missing peripheral: before this, a sketch that printed more than the 64-byte
transmit buffer stopped running. `SimEngine.drainSerialOutput()` returns each board's serial output since the last
call (the newest 16 KiB of it per board, if the host drains less often; `ArduinoMcu.serialOutputDropped()` counts
what was dropped) and `SimEngine.writeSerialInput(id, text)` types into a board's serial port. EEPROM contents survive a board
reset, as on the real chip; an I2C bus with no modelled part answers every address with a NACK.
