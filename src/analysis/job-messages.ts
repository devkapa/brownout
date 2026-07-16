/**
 * Typed message protocol between the analysis WebWorker and the main thread.
 *
 * Kept separate from sim/messages.ts so the analysis worker can be loaded
 * independently without pulling in the live-sim types.
 *
 * AnalysisWorkerIn  — messages the main thread sends to the worker.
 * AnalysisWorkerOut — messages the worker sends back.
 *
 * WHY AnalysisAnyResult in the result message:
 *   The worker now handles four job kinds (dc-sweep, temp-sweep, ac-sweep,
 *   monte-carlo).  Using the union keeps the protocol type-safe without a
 *   parallel message type per job kind — the receiver discriminates on
 *   result.spec.kind via isAcSweepResult() / isMonteCarloResult().
 */

import type { SimCircuit } from "../sim/engine/sim-engine.js";
import type { AnalysisJobSpec, AnalysisAnyResult } from "./jobs.js";

export type AnalysisWorkerIn =
  | { type: "run"; spec: AnalysisJobSpec; circuit: SimCircuit }
  | { type: "cancel" };

export type AnalysisWorkerOut =
  | { type: "progress"; index: number; total: number }
  | { type: "result"; result: AnalysisAnyResult }
  | { type: "error"; message: string };
