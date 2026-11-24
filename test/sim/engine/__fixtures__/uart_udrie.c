// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Sends "ABCDE" from the data-register-empty interrupt, as HardwareSerial does, then turns UDRIE off.
#include <avr/io.h>
#include <avr/interrupt.h>
static const char msg[] = "ABCDE";
static volatile unsigned char idx;
ISR(USART_UDRE_vect) {
  UDR0 = msg[idx++];
  if (idx == 5) UCSR0B &= ~_BV(UDRIE0);
}
int main(void) {
  UCSR0A = _BV(U2X0); UBRR0 = 16; UCSR0C = 3 << UCSZ00;
  UCSR0B = _BV(TXEN0) | _BV(UDRIE0);
  sei();
  for (;;);
}
