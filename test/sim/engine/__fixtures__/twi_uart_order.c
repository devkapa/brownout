// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// A START asked for straight after a STOP waits out the bus-free time before it goes on the bus; a serial byte sent
// in that wait starts first. Bus events are reported in the order they happen on the wires.
#include <avr/io.h>
int main(void) {
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0C = 3 << UCSZ00; UCSR0B = _BV(TXEN0);
  TWSR = 0; TWBR = 72;
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); while (!(TWCR & _BV(TWINT)));
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); while (TWCR & _BV(TWSTO));
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  UDR0 = 'u';
  while (!(TWCR & _BV(TWINT)));
  for (;;);
}
