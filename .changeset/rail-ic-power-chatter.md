---
"brownout": patch
---

A supply-gated chip that cannot power up on its own rail now says so. When a chip's own pins load its supply back below its power-on level as soon as it powers (a 555 with TRIG, THR and DIS on a current-limited rail, say), no state holds at any step size: powered, the rail falls under the level, and unpowered, it rises past it again. The hosts used to retry down to their 10 ns floor and stop the clock with nothing said. The engine still stops there, since no physical state lets it go on, but it now latches an engine warning, `rail_ic_power_chatter`, naming the chip, its rail (the supply on its vcc net, or the net) and its power-on level. Hosts that forward engine warnings show it with no change on their side.

No reading or class changes: every classic suite (2054 tests, dense and sparse) passes, and the classic corpus sweep (the brownout fixtures, the synthetic fixtures and devolt's core corpus, 159 circuits at 0, D/4, D/2 and D) reads bit for bit as before, with the same stops and warnings. The warning latches only on a step that fails at the floor with the chip's power alternating, which stopped the clock before this change too.
