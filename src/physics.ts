/**
 * Physics sidecar entry: the battery chemistry model (OCV/SoC Thevenin with
 * depletion and temperature effects), the one-pole package thermal model
 * (profiles, shutdown hysteresis, derating), the servo shaft model, and the
 * telemetry builder that snapshots all of them from a SimEngine.
 *
 * These modules are engine-integrated (SimEngine drives them every accepted
 * step) but their types and pure helpers are useful standalone — e.g.
 * building a discharge curve without a circuit. Thermal LED profiles resolve
 * against the INJECTED part library via thermalProfilesByCatalogUid(); the
 * module-init constant simcore had cannot exist here because a constant
 * cannot react to setPartLibrary().
 */

export * from "./sim/battery-physics.js";
export * from "./sim/thermal-physics.js";
export * from "./sim/servo-physics.js";
export { buildPhysicsTelemetry } from "./sim/physics-telemetry.js";
