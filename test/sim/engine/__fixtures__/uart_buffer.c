// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Bare-metal USART at 115200 (U2X, UBRR 16): writes 'A', 'B', 'C' back to back, polling UDRE as
// HardwareSerial does, and marks PORTB after each write is accepted, then 4 once TXC says the
// last frame has left. Clears TXC by writing UCSRA as HardwareSerial does.
#include <avr/io.h>
int main(void) {
  DDRB = 0xff;
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0B = _BV(TXEN0); UCSR0C = 3 << UCSZ00;
  const char *s = "ABC";
  for (unsigned char i = 0; i < 3; i++) {
    while (!(UCSR0A & _BV(UDRE0)));
    UDR0 = s[i];
    UCSR0A = (UCSR0A & (_BV(U2X0) | _BV(MPCM0))) | _BV(TXC0);
    PORTB = i + 1;
  }
  while (!(UCSR0A & _BV(TXC0)));
  PORTB = 4;
  for (;;);
}
