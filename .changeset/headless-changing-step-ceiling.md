---
"brownout": minor
---

`HeadlessRunner` caps error-controlled steps at 2 ms while the circuit is changing, and keeps the 10 ms `H_MAX` while it sits still. Backward Euler lags an RC curve by about half a step, and the local error tolerance alone let slow RC timing run at 10 ms steps: a textbook 555 astable (4.7 kohm, 10 kohm, 10 uF) ran 1.5% slow at 1x speed and 2.2% at 4x, and its period wandered by ±0.6 ms from cycle to cycle. With the cap it runs 0.6% slow with a ±0.04 ms spread. "Changing" means the last error estimate, extrapolated to a full 10 ms step (local error scales as h^(order+1)), would spend more than 10% of the tolerance; a rejected step counts as changing. A flat 2 ms ceiling was measured and rejected: it cost static logic and memory boards 3-4x the solves for no accuracy gain. Fixed-step runs and MCU circuits are unaffected.
