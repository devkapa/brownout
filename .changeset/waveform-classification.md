---
"brownout": patch
---

A `signal_gen` whose `waveform` is not one of the eight names the engine knows now gets a sine's step limit in `HeadlessRunner`. The engine plays any other value as a sine (`"Sine"`, `"SQUARE"`, a typo, a number), but the runner picked the step limit from the raw string, so those sines got none: `"Sine"` at 1 kHz ran 22 steps over 20 ms, up to 8.2 ms each, and read above mid-scale for 1.1 ms instead of 10 ms. The runner now reads the waveform through the engine's own parse (`parseSignalGenParams`), so such a generator steps exactly like `"sine"`: 812 steps, 25 us at most. The eight recognised names step exactly as before.
