---
"brownout": minor
---

New part: the BMP280 pressure and temperature sensor module on I2C, kind `bmp280`, a target on an Arduino board's
TWI like the SSD1306. Its `temperature` (degC) and `pressure` (hPa) parameters are the ambient values it measures,
and a parameter edit reaches the next conversion without a reset. A sketch reads it through the chip's own
registers: chip id 0x58, the datasheet's example trimming parameters, ctrl_meas and config, forced and normal modes
with the datasheet's typical measurement time and standby periods, 16- to 20-bit resolution with oversampling, the
IIR filter, data shadowing across a burst read and the B6h soft reset. Each conversion produces the raw readings that
the datasheet's compensation maps back to the ambient values, so Adafruit_BMP280 2.6.8 reads them to 0.01 degC and
under a pascal; a test pins that with a sketch built by the in-browser avr-gcc. The address is 0x77 by default (the
`address` parameter takes 0x76). I2C parts now also run on the clock of the board that addresses them.
