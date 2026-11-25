// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Bare-metal USART receive at 115200 (U2X, UBRR 16). Sets PORTB to 1 the moment RXC first reads set, then leaves the
// receiver alone for well over four frames, so four bytes sent back to back overrun the two-byte receive FIFO. Then
// reads until RXC clears, storing UCSR0A and UDR0 for each byte from 0x0301 and the count at 0x0300, and sets PORTB
// to 2.
#include <avr/io.h>
#define RES ((volatile unsigned char *)0x300)
int main(void) {
  DDRB = 0xff;
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0C = 3 << UCSZ00; UCSR0B = _BV(RXEN0);
  while (!(UCSR0A & _BV(RXC0)));
  PORTB = 1;
  for (volatile unsigned int i = 0; i < 3000; i++);
  unsigned char n = 0;
  while ((UCSR0A & _BV(RXC0)) && n < 6) { RES[1 + 2 * n] = UCSR0A; RES[2 + 2 * n] = UDR0; n++; }
  RES[0] = n;
  PORTB = 2;
  for (;;);
}
