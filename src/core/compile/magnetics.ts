/**
 * Inductors, the coupled inductor and the crystal (W1h item H4a). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H4a lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H4a replaces the body of
 * `compileMagneticsKind` and fills `MAGNETICS_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (inductor, coupled_inductor, crystal). */
export const MAGNETICS_KINDS: ReadonlySet<string> = new Set(["inductor", "coupled_inductor", "crystal"]);

export function compileMagneticsKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const MAGNETICS_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
