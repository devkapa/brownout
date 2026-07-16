/**
 * Advance an analysis engine by one accepted step.
 *
 * `SimEngine.step()` deliberately reports Newton/MNA failure through
 * `lastConverged` instead of throwing because the live worker can retry with a
 * smaller timestep. Analysis runners use a fixed, reproducible timestep, so
 * accepting that failed iterate would silently contaminate every later sample.
 *
 * Keep this policy in one place: snapshot, attempt, restore on either a thrown
 * exception or non-convergence, then reject the analysis point with enough
 * diagnostics for the UI to explain what happened.
 */

import type { SimEngine } from "../sim/engine/sim-engine.js";

export class AnalysisConvergenceError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "AnalysisConvergenceError";
  }
}

function finiteDiagnostic(value: number): string {
  return Number.isFinite(value) ? String(value) : "non-finite";
}

export function stepAnalysisEngine(
  engine: SimEngine,
  h: number,
  context: string,
): void {
  const snapshot = engine.saveState();
  const startTime = engine.simTime;

  try {
    engine.step(h);
  } catch (cause) {
    engine.restoreState(snapshot);
    throw new Error(
      `Analysis stopped during ${context}: the simulator could not advance at ` +
      `${String(startTime)} s. The attempted step was rolled back and no invalid ` +
      `data was used. Technical detail: engine.step threw for h=${String(h)} s: ${String(cause)}`,
    );
  }

  if (engine.lastConverged) return;

  const diagnostics = [
    `iterations=${String(engine.lastIters)}`,
    `singular=${String(engine.lastMatrixSingular)}`,
    `illConditioned=${String(engine.lastMatrixIllConditioned)}`,
    `relativeResidual=${finiteDiagnostic(engine.lastRelativeResidual)}`,
  ].join(", ");

  engine.restoreState(snapshot);
  throw new AnalysisConvergenceError(
    `Analysis stopped because ${context} did not converge at ${String(startTime)} s. ` +
    `The attempted step was rolled back, this analysis point was rejected, and no ` +
    `invalid values were used. Check for conflicting ideal sources, floating nodes, ` +
    `or extreme component values. Technical detail: h=${String(h)} s; ${diagnostics}`,
  );
}
