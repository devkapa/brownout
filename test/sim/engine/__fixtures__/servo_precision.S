/*
 * Minimal ATmega328P firmware used by the servo PinEvent integration test.
 * Arduino D9 is PORTB bit 1. The HIGH delay is 6,147 four-cycle loop
 * iterations, giving 24,591 cycles = 1.5369375 ms between captured PORT
 * writes at 16 MHz. The deliberately off-grid width maps near 96.65 degrees;
 * a 100 us outer-step sampler could report only 90 or 108 degrees instead.
 */

.section .text
.global __vectors

__vectors:
  rjmp reset

reset:
  ldi r16, 0x02
  out 0x04, r16

loop:
  out 0x05, r16
  ldi r24, lo8(6147)
  ldi r25, hi8(6147)

high_delay:
  sbiw r24, 1
  brne high_delay

  ldi r17, 0x00
  out 0x05, r17
  ldi r24, lo8(50000)
  ldi r25, hi8(50000)

low_delay:
  sbiw r24, 1
  brne low_delay

  rjmp loop
