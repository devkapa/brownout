/**
 * Small-signal AC entry: linearizes at the DC operating point (Wave A5) and
 * sweeps a complex solve per frequency point.
 *
 * AcSystem/AcStampSurface are exported because registered device models can
 * implement an AC stamp hook; a third-party model author needs the stamp
 * surface type. The driven-transient "large-signal AC" sweep is a different
 * tool with different honesty semantics and lives in "brownout/analysis"
 * (runAcSweep).
 */

export { runSmallSignalAc } from "./sim/engine/ac-analysis.js";
export type {
  SmallSignalAcNetOutput,
  SmallSignalAcOptions,
  SmallSignalAcResult,
} from "./sim/engine/ac-analysis.js";
export { AcSystem } from "./sim/engine/ac-system.js";
export type { AcStampSurface } from "./sim/engine/ac-system.js";
