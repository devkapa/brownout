---
"brownout": patch
---

`spdt_switch`'s reported current now follows the contact actually in use — `com` to `b` at position 0, `com` to `c` otherwise — instead of always reading `com` to `b`. A probe on a switch left at position 1 used to read a nonsense value (5000 A on the fixture that caught this) because it measured the unselected, floating throw; it now reads the real current through whichever contact is closed. A component declared with fewer than 3 pins (so `stamp` never stamps a contact at all) now also reports 0 instead of a similarly nonsensical current measured through a node the switch never touched.
