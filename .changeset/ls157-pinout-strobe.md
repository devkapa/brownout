---
"brownout": patch
---

74LS157: the strobe (/G, catalog pin "/oe") now forces every Y output LOW while it is HIGH, as the totem-pole part does, instead of leaving the outputs undriven (that is the 74LS257). The default catalog lists the pins in data-sheet order, so pins 9 to 15 are 3Y, 3B, 3A, 4Y, 4B, 4A, /G. Pin ids are unchanged.
