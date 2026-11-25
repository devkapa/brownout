// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// One byte at 1200 baud (U2X, UBRR 1666): a frame is (1666 + 1) x 8 x 10 = 133360 cycles.
#include <avr/io.h>
int main(void) {
  UCSR0A = _BV(U2X0); UBRR0 = 1666; UCSR0C = 3 << UCSZ00; UCSR0B = _BV(TXEN0);
  while (!(UCSR0A & _BV(UDRE0))); UDR0 = 'x';
  for (;;);
}
