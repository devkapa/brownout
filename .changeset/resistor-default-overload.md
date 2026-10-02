---
"brownout": patch
---

A resistor with no `resistance` param can now overload, and a fuse with no `iRating` param can now trip. The engine conducted it as 1 kohm, its fallback for a missing value, but the overload check read the missing value as 0 ohm and so as 0 W: 50 V across it dissipated 2.5 W through the 0.25 W default rating and it never failed open. It now uses the same 1 kohm, so it latches `resistor_overload` in about 0.1 s at that power. A fuse with no `iRating` read as 0 A and never tripped; it now uses the catalog's 1 A default. Parts that set the param are unchanged, and an explicit `iRating` of 0 still means no limit.
