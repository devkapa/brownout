// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// TWI flags at 100 kHz (TWBR 72), marked on PORTB. Bit 0: TWINT reads clear out of reset (TWCR resets to 0). Bit 1:
// after a START completes, a TWCR write with TWINT 0 (changing only TWEA) leaves TWINT set, since only writing a one
// clears it. Bit 2: after a STOP, TWINT stays clear and TWSR reads 0xF8. Then START, STOP and START again with no
// pause, so the last START has to wait out the bus-free time after the STOP. Bit 3 once that START is done.
#include <avr/io.h>
int main(void) {
  DDRB = 0xff;
  TWSR = 0; TWBR = 72;
  if (!(TWCR & _BV(TWINT))) PORTB |= 1;
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); while (!(TWCR & _BV(TWINT)));
  TWCR = _BV(TWEN) | _BV(TWEA);
  if (TWCR & _BV(TWINT)) PORTB |= 2;
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); while (TWCR & _BV(TWSTO));
  for (volatile unsigned char i = 0; i < 20; i++);
  if (!(TWCR & _BV(TWINT)) && (TWSR & 0xf8) == 0xf8) PORTB |= 4;
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); while (!(TWCR & _BV(TWINT)));
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); while (TWCR & _BV(TWSTO));
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); while (!(TWCR & _BV(TWINT)));
  PORTB |= 8;
  for (;;);
}
