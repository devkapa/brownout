// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Bare-metal I2C at 100 kHz (TWBR 72), then 400 kHz (TWBR 12), each: START, address 0x3C write
// (NACKed on an empty bus), STOP, polled as Wire does. Marks each phase on PORTB so a test can read
// the cycle each one ended on. Then idles.
#include <avr/io.h>
static void twi_wait(void) { while (!(TWCR & _BV(TWINT))); }
static void transaction(void) {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN); twi_wait();
  TWDR = 0x3C << 1; TWCR = _BV(TWINT) | _BV(TWEN); twi_wait();
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); while (TWCR & _BV(TWSTO));
}
int main(void) {
  DDRB = 0xff;
  TWSR = 0;
  TWBR = 72; PORTB = 1; transaction(); PORTB = 2;
  TWBR = 12; PORTB = 3; transaction(); PORTB = 4;
  for (;;);
}
