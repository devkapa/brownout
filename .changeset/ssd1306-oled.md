---
"brownout": minor
---

New part: the SSD1306 128x64 (or 128x32) I2C OLED module, kind `ssd1306`. The controller is a target on an Arduino
board's TWI when the module's SDA and SCL share nets with the board's (A4 and A5 on the Uno and Nano), so it
acknowledges its address (0x3C, or 0x3D with the jumper moved) and takes bytes at the bus timing the board's TWI
already has. It parses the control byte, the command set with its arguments, GDDRAM in page, horizontal and
vertical addressing, start line, offset, multiplex ratio, segment remap, COM scan direction, contrast, inverse and
entire-display-on, and lights only with display-on and the charge pump. `getDisplayState()` reports the panel's
1-bit image (`kind: "ssd1306"`). Scrolling is accepted but not drawn. A sketch built against Adafruit_SSD1306
2.5.13 runs unchanged and its frame is pinned by a test. The module draws about 20 mA lit and 10 uA asleep, and
power returning is a power-on reset.
