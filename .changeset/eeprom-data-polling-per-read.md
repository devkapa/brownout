---
"brownout": patch
---

While a 28C16 or 28C256 is busy with a write, the DATA-polling bit (I/O6) now toggles once per read, which is what the datasheet describes ("successive reads toggle I/O6"). The engine flipped it on every solver step instead, so a read that stayed enabled across several steps saw I/O6 jump around with the step size, and two reads far apart could land on the same value by accident. Now each new read (each time /OE and /CE both go low) flips it once: the first read of a pending write sees I/O6 high, the next sees it low, and holding /OE low does not move it. The bit is also no longer one step late on the first step of a read.
