---
"brownout": patch
---

A new part: the 74LS153 dual 4-line to 1-line data selector/multiplexer (`74ls153`, catalog uid `ic-74ls153`, DIP-16). Select inputs A (least significant) and B are shared by both sections, and each section routes its data input C0 to C3 numbered B×2+A to its Y output. Each section has its own active-low strobe, /1G or /2G: a section whose strobe is HIGH drives its Y LOW, as the real chip's totem-pole outputs do (the SN74LS153 data sheet's function table); it does not release the pin, which is what the 74LS253 does. The pinout follows the data sheet: /1G, B, 1C3, 1C2, 1C1, 1C0, 1Y and GND on pins 1 to 8, then 2Y, 2C0, 2C1, 2C2, 2C3, A, /2G and VCC on pins 9 to 16. It is an LS-TTL part with the same electrical specs and default 20 ns propagation delay as the engine's other 74LS combinational chips, and shares their stamping, output-sag, small-signal and delay paths, so no existing part changes.
