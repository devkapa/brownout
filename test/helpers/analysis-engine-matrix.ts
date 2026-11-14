/**
 * The analysis suites' engine matrix (W2e): every suite that runs an analysis
 * runs once with the engine pinned to classic and once on `auto` with the
 * core's accuracy gate widened to everything the compiler can produce
 * (`CORE_IMPLEMENTED_KINDS`). The shipped gate (`CORE_SUPPORTED_KINDS`) leaves
 * out the implemented kinds that are not promoted yet, so without widening a
 * suite using one of them would never reach the core on `auto` and its
 * "auto" column would silently be a second classic column.
 *
 * `expectedEngine` is what makes that impossible to miss: a matrix helper
 * asserts the result's `engine` against it, so a supported circuit that
 * quietly fell back (core not initialised, a compile reason) fails the test
 * instead of passing on classic.
 */

import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { unsupportedCoreKinds } from "../../src/core/select.js";
import type { AnalysisEngineChoice, AnalysisEngineKind } from "../../src/analysis/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

export const ENGINE_MODES = ["classic", "auto"] as const;
export type EngineMode = (typeof ENGINE_MODES)[number];

/** The options a run function takes for `mode`. */
export function matrixOptions(mode: EngineMode): {
  engine: AnalysisEngineChoice;
  coreSupportedKinds: ReadonlySet<string>;
} {
  return { engine: mode, coreSupportedKinds: CORE_IMPLEMENTED_KINDS };
}

/** Which engine `mode` must land on for `circuit`: the core only on `auto` and only when every kind is core-supported. */
export function expectedEngine(mode: EngineMode, circuit: SimCircuit): AnalysisEngineKind {
  if (mode === "classic") return "classic";
  return unsupportedCoreKinds(circuit, CORE_IMPLEMENTED_KINDS).length === 0 ? "core" : "classic";
}
