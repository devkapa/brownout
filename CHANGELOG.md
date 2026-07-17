# brownout

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
