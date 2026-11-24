---
"brownout": minor
---

Arduino boards keep the ATmega328P's bus timing where avr8js has none, in the classic engine and the AVR core
alike. An I2C step now takes its bus time at the SCL rate TWBR and the TWSR prescaler set (nine SCL periods per
address or data byte, half a period for a START from idle, one and a half for a repeated START, one for a STOP)
instead of finishing the instant it starts, so Wire's transactions take as long as on hardware; TWSR's status bits
are read-only, as on the chip. The USART transmitter is double-buffered: a second byte waits in UDR while the first
is on the wire and starts the moment it ends, UDRE stays set while the shift register takes the first byte, and
writing UCSRA no longer clears UDRE and RXC. Serial output and bus events report a byte as its frame starts. I2C bus
events now carry their length in `cycles`. SPI is unchanged: avr8js already times it at the SCK rate.

The remaining datasheet differences are fixed the same way, in both engines. A write to UDR while it is full is
ignored, and starting a frame no longer clears TXC. The receiver raises RXC at the first stop bit's middle sample,
half a bit before the frame ends, keeps a two-frame FIFO with a third frame in the shift register, sets DOR on the
next frame read when a fourth start bit overruns it, and flushes on disabling RXEN; typed serial input now waits
for RXEN instead of being stranded when it arrives first. TWINT is clear out of reset, a STOP no longer raises it
(TWSR reads 0xF8 with no interrupt), a write of zero to it leaves it set, and neither it nor UDRE clears when its
interrupt is taken, so a handler that leaves either set runs again. A START after a STOP waits out the
bus-free time (4.7 us at SCL up to 100 kHz, 1.3 us above). `ArduinoMcu.step` carries each step's instruction
overshoot into the next, so a board's clock no longer runs ahead of the circuit's (about 0.005% before), and
`SimEngine.drainMcuBusEvents` dates events from each board's own clock, so time a board spends unpowered or paused
after an event no longer moves it; they are reported in the order they reach the wires. A breakpoint the program
reaches exactly at a step's end now stops it. `MicrocontrollerCore` gains an optional `clock()`.
