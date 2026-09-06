---
"brownout": minor
---

Add latched engine warnings beside failures: `SimWarning` / `SimWarningCode`, `SimEngine.getWarnings()`, `SimEngine.takeNewWarnings()`, and `DeviceContext.hasWarning` / `recordAccumulatedWarning` for device models. Two conditions ship: `transistor_overcurrent` (a BJT or MOSFET whose committed collector/drain current exceeds its catalog `i_c_max` for 100 us of accepted simulated time, reporting the measured current against the rating) and `missing_flyback` (an inductor, relay coil, or motor winding switched by a transistor or mechanical switch with no diode-like part on its switch node, evaluated from the loaded graph). Warnings never alter a stamp, so every existing trajectory is unchanged; they latch once per condition per component, carry across `load()` for a same-id part, roll back with `restoreState()`, and clear with `resetFailures()`. The `warn` worker message gains an optional `componentId`, and `HeadlessSnapshot` carries `warnings`.
