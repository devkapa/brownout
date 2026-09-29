---
"brownout": minor
---

A passive buzzer now detects its tone from a 0-5 V drive. The detector counted sign changes of the terminal voltage around 0 V, so the square wave the part's own help recommends (a `clock_gen` or a PWM pin, which never goes negative) never registered: `detectedHz` stayed 0 and `sounding` stayed 0. Crossings are now taken around the drive's own midpoint, from an envelope that follows each new extreme and relaxes over 0.1 s, with a Schmitt band of 10% of the swing. The frequency is timed over a full period (two crossings in the same direction), so a PWM pin at 25% duty reads its drive frequency rather than alternating between two wrong ones. Sine and bipolar drives still read the same frequency.

The readout also follows the drive now. A tone clears two periods after the last crossing, so a buzzer whose drive stops falls silent. Before, the last frequency latched forever. A reading counts only inside the audible band (20 Hz to 20 kHz, previously 1 Hz to 100 kHz), so one press of a button across a passive buzzer is two clicks rather than a 2 Hz tone. Swings under 50 mV peak-to-peak (the speaker's signal-present floor) are silence.

The envelope (`envHi`/`envLo` in the buzzer's IC state) is skipped by the step-error estimate. It is readout-only and moves on every step by an amount that depends on h, so comparing it would let the readout shrink the step. The crossing times are still compared.

Physics telemetry reports the state as `modelState`: `{ kind: "buzzer", buzzerType, sounding, toneHz? }`, with `toneHz` present only while a passive buzzer detects a tone, and `{ kind: "speaker", sounding }`. Both are `behavioral-estimate` quality, read through `engine.getIcState`.
