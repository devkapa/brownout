---
"brownout": minor
---

The Arduino Uno's board geometry is now real size for hosts that draw 19 px per
0.1": the PCB is 513 x 398 px (68.58 x 53.34 mm) and every pin offset, the
bounds, the USB toggle and the reset button scale with it (about 1.95x).
`ARDUINO_UNO_ART_GROWTH` gives hosts the factor for their own hand-placed art.
Pin ids are unchanged, so wires keep their ends; only the offsets move.
