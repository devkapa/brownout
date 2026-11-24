---
"brownout": patch
---

The new parts follow their datasheets in five more places. A DHT that loses power or is reset in the middle of an
answer lets go of its pin (it held the line low). A DHT11 rounds its temperature to tenths before splitting it, so
23.96 degC reads 24.0 (it read 23.0). A DHT read sooner than its minimum interval after the reading it last measured
behind (2 s on the DHT22, 1 s on the DHT11, timed from release to release) sends that measurement again and does not
measure after it (it measured after every reading); a board reset, or a board new to the part (a re-upload
rebuilds the board), tells it through the new optional `PinTarget.boardReset()` that the time since is unknown, so it
measures. A BMP280 asked for forced mode while a forced
conversion is still running carries on with that conversion (it restarted it). An SSD1306 module whose panel is
supplied from outside (its new `supply` parameter set to `"external"`, as `SSD1306_EXTERNALVCC` sketches expect)
lights with the charge pump off; the common module, which has only the pump, still stays dark when a sketch turns
the pump off.

The AVR core's port of these parts (feature `avr`, not in the shipped WebAssembly module yet) has the same fixes and
lock-steps with them; its test ABI's `avr_attach_ssd1306` takes the external-supply flag as a third argument.
