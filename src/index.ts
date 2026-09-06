/**
 * Brownout core surface: SimEngine, the circuit domain types, the device
 * registry, the linear-system backends, and the injectable part library.
 *
 * The rest of the engine ships behind sibling entry points so consumers pull
 * only what they use: "brownout/spice" (netlist interop), "brownout/ac"
 * (small-signal AC), "brownout/analysis" (offline runners), "brownout/physics"
 * (battery/thermal/servo sidecars + telemetry), "brownout/mcu" (MCU cores and
 * emulator registration), "brownout/host" (host adapters + the step-controller
 * contract).
 *
 * Deliberately NOT exported from THIS barrel: the host-loop tooling. The core
 * is transport-free — it must stay importable where there is no loop and no
 * clock — so anything that owns a loop or encodes a policy about time lives
 * behind "./host" instead. The adaptive-step controller (estimateStepError,
 * nextStepFactor) and HeadlessRunner ship there as of phase B3. Still
 * unexported anywhere, pending an adapter that freezes their shape: the worker
 * protocol messages and the meter/scope/trace DSP. The one exception to the
 * rule is the ArduinoState readout type, which appears in SimEngine's public
 * getArduinoState() signature.
 *
 * Naming note: circuit/types.ts and engine/graph.ts both declare a `Net`.
 * The circuit-document shape keeps the bare name (this barrel re-exports the
 * whole domain-type module); the solver's resolved-net shape is exported as
 * `EngineNet` to avoid the collision simcore's root barrel had to suppress.
 */

// Circuit domain types + catalog identity
export * from "./circuit/types.js";
export * from "./circuit/arduino.js";
export * from "./circuit/catalog-resolver.js";

// Injectable part library (the proprietary-catalog seam)
export * from "./parts/part-library.js";

// Engine core
export { SimEngine } from "./sim/engine/sim-engine.js";
export type {
  SimCircuit,
  DcOperatingPointResult,
  StateSnapshot,
  ErrorStateSnapshot,
  SolverDiagnosticsSnapshot,
  SimFailure,
  SimFailureKind,
  SimWarning,
  SimWarningCode,
  BatteryRuntimeReadout,
  ThermalRuntimeReadout,
  LogicThresholds,
  IcPowerInfo,
} from "./sim/engine/sim-engine.js";
export { buildNets } from "./sim/engine/graph.js";
export type { Net as EngineNet, PinKey } from "./sim/engine/graph.js";
export type { ArduinoState } from "./sim/messages.js";

// Device registry (third-party device models register through this)
export * from "./sim/engine/device-registry.js";

// Linear-system backends
export * from "./sim/engine/linear-system.js";
export { MNA } from "./sim/engine/mna.js";
export type { MnaSolveInfo } from "./sim/engine/mna.js";
export { SparseMNA } from "./sim/engine/sparse-mna.js";
export * from "./sim/engine/newton.js";
