/**
 * The shift registers 74hc595 and 74hc165 (W1h item H2a). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H2a lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H2a replaces the body of
 * `compileShiftIc` and fills `SHIFT_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (74hc595, 74hc165). */
export const SHIFT_KINDS: ReadonlySet<string> = new Set(["74hc595", "74hc165"]);

export function compileShiftIc(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const SHIFT_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
