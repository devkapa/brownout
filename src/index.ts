/**
 * Brownout core surface: SimEngine, the circuit domain types, the device
 * registry, the linear-system backends, and the injectable part library.
 *
 * The rest of the engine ships behind sibling entry points so consumers pull
 * only what they use: "brownout/spice" (netlist interop), "brownout/ac"
 * (small-signal AC), "brownout/analysis" (offline runners), "brownout/physics"
 * (battery/thermal/servo sidecars + telemetry), "brownout/mcu" (MCU cores and
 * emulator registration).
 *
 * Deliberately NOT exported anywhere yet: the host-loop tooling
 * (adaptive-step, worker protocol messages, meter/scope/trace DSP). Those are
 * the transport-adapter contract and land with the "./host" entry in phase
 * B3 — exporting them from the core now would freeze their shape before the
 * host adapters exist. The one exception is the ArduinoState readout type,
 * which appears in SimEngine's public getArduinoState() signature.
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
