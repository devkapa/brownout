---
"brownout": minor
---

New part: the DHT22 (AM2302) and DHT11 humidity and temperature modules, kind `dht` with a `variant` parameter (22
or 11), DATA's 10 kOhm pull-up on the module. The sensor is a target on whichever Arduino pin its DATA shares a net
with: `ArduinoMcu.setPinTargets()` hands it the board's drive on that pin at the cycle it changes, and it pulls the
board's input low at the cycles of its reply (80 us low, 80 us high, then 40 bits of a 50 us low and a 27 or 70 us
high). `humidity` and `temperature` are what it measures, in the DHT22's tenths or the DHT11's whole percent and tenth
degrees. As on the parts, a reading reports the measurement made after the previous one, the part ignores the line
for its first second of power, and a start shorter than 0.8 ms (DHT22) or 18 ms (DHT11) gets no answer. Adafruit's DHT
sensor library 1.4.6 reads both, pinned by tests with sketches built by the in-browser avr-gcc. The device context's
`i2cDevice` (added with the SSD1306, unreleased) is now `boardPart`, as it also returns pin targets.
