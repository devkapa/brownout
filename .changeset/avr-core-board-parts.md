---
"brownout": minor
---

The AVR core (feature `avr`) carries the board's new parts, so I2C and DHT sketches run the same on it as on the
classic engine: `core/src/mcu/avr/ssd1306.rs`, `bmp280.rs` and `dht.rs` port the SSD1306, BMP280 and DHT models,
and `parts.rs` wires them as `ArduinoMcu` does (I2C targets answering the timed TWI by address, a DHT as a pin target
that hears the board's drive edges and pulls its input low from clock events). The real Adafruit SSD1306, BMP280 and
DHT sketches read them in the core's own tests, and the lock-step harness now runs those images with the parts on
both sides, comparing acknowledged addresses, bytes read back, the DHT's answer and each part's state. A pin target
taken off a board now gives its pin back to the circuit's level when it was holding it low (it stayed low).
