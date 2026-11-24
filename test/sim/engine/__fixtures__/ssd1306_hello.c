// Compiled with: avr-gcc -mmcu=atmega328p -Os, then avr-objcopy -O ihex.
// Bare-metal I2C at 400 kHz (TWBR 12) to an SSD1306 at 0x3C, as Adafruit_SSD1306 drives it: the 128x64 init
// sequence, a column and page window over the whole panel, then a frame with three pixels lit: (0, 0), (5, 7) and
// (127, 9). Each step is polled as Wire does; PORTB counts the transactions the panel acknowledged, then 0x80 once
// the frame is sent.
#include <avr/io.h>
static unsigned char twi(unsigned char twcr) {
  TWCR = twcr; while (!(TWCR & _BV(TWINT)));
  return TWSR & 0xf8;
}
static void start(void) { twi(_BV(TWINT) | _BV(TWSTA) | _BV(TWEN)); }
static unsigned char send(unsigned char b) { TWDR = b; return twi(_BV(TWINT) | _BV(TWEN)); }
static void stop(void) { TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN); while (TWCR & _BV(TWSTO)); }
static void commands(const unsigned char *c, unsigned char n) {
  start();
  if (send(0x3c << 1) == 0x18) PORTB++;
  send(0x00);
  while (n--) send(*c++);
  stop();
}
static const unsigned char init[] = {
  0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12, 0x81, 0xcf, 0xd9,
  0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0x2e, 0xaf,
};
static const unsigned char window[] = { 0x21, 0, 127, 0x22, 0, 7 };
int main(void) {
  DDRB = 0xff;
  TWSR = 0; TWBR = 12;
  commands(init, sizeof init);
  commands(window, sizeof window);
  start();
  if (send(0x3c << 1) == 0x18) PORTB++;
  send(0x40);
  for (unsigned int i = 0; i < 1024; i++) {
    unsigned char b = 0;
    if (i == 0) b = 0x01;
    if (i == 5) b = 0x80;
    if (i == 128 + 127) b = 0x02;
    send(b);
  }
  stop();
  PORTB |= 0x80;
  for (;;);
}
