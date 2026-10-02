---
"brownout": patch
---

A part's thermal warnings now include its profile's assumptions from the moment the circuit loads. Every step publishes the profile's own caveats (the first-order thermal model, a headline rating that needs a heatsink, an assumed shutdown restart point) together with that step's derating warnings, but a part that starts cold on `load()` carried only the derating warnings. The profile's caveats showed up only once the 1 ps operating-point solve at the end of `load()` had converged, so on a circuit whose seed solve is rejected `getThermalState` reported none of them until the first accepted step. The cold start now publishes the same set.
