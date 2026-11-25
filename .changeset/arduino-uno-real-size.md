---
"brownout": minor
---

The Arduino Uno's board geometry is now real size for hosts that draw 19 px per
0.1": the PCB is 513 x 398 px (68.58 x 53.34 mm) and every pin offset, the
bounds, the USB toggle and the reset button scale with it (about 1.95x).
`ARDUINO_UNO_ART_GROWTH` gives hosts the factor for their own hand-placed art.
Pin ids are unchanged, so wires keep their ends; only the offsets move.

The header pins now follow the Uno R3 drawing on an exact 19 px (0.1") grid,
with the 160 mil step between D7 and D8; `ARDUINO_UNO_HEADER_HOLES` lists every
header hole, modelled or not, so hosts can draw the art's holes from it.
