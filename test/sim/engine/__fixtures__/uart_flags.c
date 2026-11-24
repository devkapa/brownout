// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Bare-metal USART at 115200 (U2X, UBRR 16). Writes 'A', 'B' and 'C' without polling: A goes straight to the shift
// register, B waits in UDR, and C, written while UDR is full (UDRE clear), is ignored, as the datasheet says. Once
// TXC says B's frame has left, writes 'D' without clearing TXC and sets PORTB bit 0 if TXC still reads set: starting
// a frame does not clear it, only a write of one or the interrupt does.
#include <avr/io.h>
int main(void) {
  DDRB = 0xff;
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0C = 3 << UCSZ00; UCSR0B = _BV(TXEN0);
  UDR0 = 'A'; UDR0 = 'B'; UDR0 = 'C';
  while (!(UCSR0A & _BV(TXC0)));
  UDR0 = 'D';
  if (UCSR0A & _BV(TXC0)) PORTB |= 1;
  for (;;);
}
