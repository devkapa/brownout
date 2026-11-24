// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// TWINT and UDRE are levels on the 328P: entering their interrupt does not clear them. A START at 100 kHz with TWIE
// set: the TWI handler marks PORTB bit 0 if TWINT reads set inside it, then turns TWIE off with TWINT written 0 (so
// it stays set) and main marks bit 1 if it still is. The UDRE handler marks bit 2 if UDRE reads set inside it and
// turns UDRIE off. Bit 3 once both have run.
#include <avr/io.h>
#include <avr/interrupt.h>
volatile unsigned char done;
ISR(TWI_vect) {
  if (TWCR & _BV(TWINT)) PORTB |= 1;
  TWCR = _BV(TWEN);
  done |= 1;
}
ISR(USART_UDRE_vect) {
  if (UCSR0A & _BV(UDRE0)) PORTB |= 4;
  UCSR0B = _BV(TXEN0);
  done |= 2;
}
int main(void) {
  DDRB = 0xff;
  TWSR = 0; TWBR = 72;
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0C = 3 << UCSZ00;
  sei();
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);
  UCSR0B = _BV(TXEN0) | _BV(UDRIE0);
  while (done != 3);
  if (TWCR & _BV(TWINT)) PORTB |= 2;
  PORTB |= 8;
  for (;;);
}
