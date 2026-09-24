/**
 * The read-only surface both engines expose, and the one a host adapter
 * reads from once it stops caring which engine is actually running. Lifted
 * out of SimEngine's own public shape (sim/engine/sim-engine.ts) rather than
 * designed fresh, so CoreEngine has no freedom to drift from what classic
 * already publishes — see the compile-time check at the bottom of
 * test/core/engine.test.ts (`const _check: EngineReadout = <a SimEngine>`),
 * which fails to typecheck the moment SimEngine's shape and this interface
 * disagree.
 *
 * This is a READ surface only: no load()/advance()/step() here. SimEngine's
 * load(circuit)/step(h) and CoreEngine's load(circuit)/advance(tEnd) differ
 * on purpose (circuit vs. compiled-program upload, relative step vs.
 * absolute advance — core/README.md's C ABI), and a host already has to
 * branch on which engine is active to drive the right one. The value of a
 * shared interface is in what it reads back afterwards, not in pretending
 * the two engines are driven identically.
 */

import type { SimFailure, SimWarning } from "../sim/engine/sim-engine.js";
import type { Net } from "../sim/engine/graph.js";
import type { DisplayInfo } from "../sim/messages.js";

export interface EngineReadout {
  /** Simulation time (s). */
  readonly simTime: number;

  /** The loaded topology's nets, in buildNets order — identical ids and order to the classic engine's. */
  readonly nets: Net[];

  /** False when the last solve did not converge; every readout below is then the last TRUSTED state, not a failed iterate. */
  readonly lastConverged: boolean;

  /** Net id -> volts. A LIVE record: a caller wanting a stable copy must spread it themselves (mirrors SimEngine.getNetV()). */
  getNetV(): Record<string, number>;

  /** Component id -> amps (pin0 -> pin1 through the element). LIVE record, same caveat as getNetV(). */
  getElementI(): Record<string, number>;

  /** Digital output levels (0 or 1) for clock/gate/DFF/555-style components. */
  readonly digitalState: Record<string, number>;

  getFailures(): Record<string, SimFailure>;

  /** Every currently latched engine warning, keyed like failures. */
  getWarnings(): Record<string, SimWarning>;

  /** Warnings latched since the previous call, in latch order; marks them announced. */
  takeNewWarnings(): SimWarning[];

  getDisplayState(): Record<string, DisplayInfo>;

  /** Resolve a component pin to its current deterministic net id, or undefined if the pin is unwired/unknown. */
  getNetIdForPin(componentId: string, pinId: string): string | undefined;
}
