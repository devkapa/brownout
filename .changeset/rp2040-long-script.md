---
"brownout": patch
---

A MicroPython program longer than about 510 bytes now runs on the Pico. `RP2040Mcu` fed the whole program, plus the Ctrl-D that runs it, into rp2040js's `USBCDC` tx FIFO in one go. That FIFO holds 512 bytes and silently drops the rest, so the tail and the Ctrl-D were lost and the program never started. The REPL feed now keeps its own unbounded queue and moves the next chunk into the FIFO each time the firmware arms a read, so programs of any length arrive whole.
