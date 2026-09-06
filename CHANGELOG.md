# brownout

## 0.3.0

### Minor Changes

- 12be168: Add latched engine warnings beside failures: `SimWarning` / `SimWarningCode`, `SimEngine.getWarnings()`, `SimEngine.takeNewWarnings()`, and `DeviceContext.hasWarning` / `recordAccumulatedWarning` for device models. Two conditions ship: `transistor_overcurrent` (a BJT or MOSFET whose committed collector/drain current exceeds its catalog `i_c_max` for 100 us of accepted simulated time, reporting the measured current against the rating) and `missing_flyback` (an inductor, relay coil, or motor winding switched by a transistor or mechanical switch with no diode-like part on its switch node, evaluated from the loaded graph). Warnings never alter a stamp, so every existing trajectory is unchanged; they latch once per condition per component, carry across `load()` for a same-id part, roll back with `restoreState()`, and clear with `resetFailures()`. The `warn` worker message gains an optional `componentId`, and `HeadlessSnapshot` carries `warnings`.

## 0.2.0

### Minor Changes

- 190865b: Add `runSmallSignalAcSweep` / `runSmallSignalAcSweepAsync` — a worker-hostable
  driver for the linearized small-signal AC analysis.

  `runSmallSignalAc` is the exact, ngspice-cross-validated transfer function, but
  its signature suits an in-process caller: it takes a live `SimEngine`, runs the
  whole frequency loop atomically, and offers no progress callback or
  cancellation. This driver adds those three things for a UI worker — a
  structured-cloneable circuit in, one `onPoint` per frequency, and cooperative
  `shouldCancel` — by composing `runSmallSignalAc` unchanged, one chunk of
  frequencies at a time with a macrotask yield between chunks. The chunking
  re-solves the (deterministic) operating point per chunk and is proven
  bit-identical to a single call. Exported from `brownout/analysis`.
