---
"brownout": patch
---

A digital output no longer reports a false `output_sag` when a one-step glitch flips it. The debounce counted consecutive sagging steps whatever the commanded direction, so a glitch on a decoder output read "commanded LOW, pin still at 5 V" and the very next step read "commanded HIGH, pin still at the glitch's low level". Those two observations reached the two-step threshold and latched a sag that no real load caused. A breadboard computer's display board did it: a 74HC74 ripple counter feeding a 74HC138 digit select shows the intermediate address for one solve on 01 to 10, and the lit digit's line raised a false `output_sag` about once every 70 ms. The count now remembers the commanded direction and restarts at 1 when it flips. A real sag, a HIGH output dragged low or a LOW output dragged high, still latches on its second step.
