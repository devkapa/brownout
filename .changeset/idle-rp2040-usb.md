---
"brownout": patch
---

A Pico sleeping in MicroPython now costs the host almost nothing. rp2040js's `USBCDC` answered every read the firmware armed on the serial OUT endpoint with an empty packet 10 µs later. TinyUSB re-armed after each one, so a program in `time.sleep()` took about 35,700 USB interrupts a second and its core was busy about 90% of the time: the default Blink ran at 0.93x real time and every Pico project read SLOW. `RP2040Mcu` now holds an armed read until it has REPL bytes to send, the way a real host does. Blink now costs about 4 ms of CPU per simulated second instead of 1,078 ms, with its pins toggling at the same times. Busy-loop programs, such as polling a button with no sleep, still keep the core fully busy, as they would on hardware.
