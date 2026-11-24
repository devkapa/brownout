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
