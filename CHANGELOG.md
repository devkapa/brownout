# brownout

## 0.4.0

### Minor Changes

- 679d00f: `HeadlessRunner` caps error-controlled steps at 2 ms while the circuit is changing, and keeps the 10 ms `H_MAX` while it sits still. Backward Euler lags an RC curve by about half a step, and the local error tolerance alone let slow RC timing run at 10 ms steps: a textbook 555 astable (4.7 kohm, 10 kohm, 10 uF) ran 1.5% slow at 1x speed and 2.2% at 4x, and its period wandered by ±0.6 ms from cycle to cycle. With the cap it runs 0.6% slow with a ±0.04 ms spread. "Changing" means the last error estimate, extrapolated to a full 10 ms step (local error scales as h^(order+1)), would spend more than 10% of the tolerance; a rejected step counts as changing. A flat 2 ms ceiling was measured and rejected: it cost static logic and memory boards 3-4x the solves for no accuracy gain. Fixed-step runs and MCU circuits are unaffected.
- 679d00f: Add step breakpoints: `SimEngine.captureStepBreakpoints`, `SimEngine.takeStepBreakpoints()` and the `StepBreakpoint` type. An NE555 switch happens inside a `step()` (the engine splits the step at the threshold crossing), but a consumer that only sees accepted states interpolates straight across the whole step, so a scope drew the 555 output edge as a ramp centred on the step, up to half a step from where it switched. With capture on, the engine records the state at the switch and, after a 10 ns backward-Euler guard sub-step, the post-switch state; a split clamped short of a crossing in the last 2% of the step carries on to one guard short of the step end so the switch still lands inside it. Breakpoints roll back with `restoreState()` (snapshots carry `stepBreakpointCount`) and clear on `load()`. Capture is off by default, and with it off every trajectory is unchanged; with it on, the guard sub-step is the only change to the numerics, independent of whether anything reads the breakpoints.

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
