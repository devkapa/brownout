/**
 * The W1h signal generator (W1h item H3a). H0 seam stub: see compile/seam.ts. The kinds decline as
 * `unsupported-kind:<kind>` until H3a lands, exactly as the default
 * `CORE_IMPLEMENTED_KINDS` gate already answers; H3a replaces the body of
 * `compileSignalGenKind` and fills `SIGNAL_GEN_LIVE_PARAM_SPECS` and owns this file.
 */

import { declineUnsupportedKind, type KindCompileContext, type SimComponent } from "./seam.js";
import type { LiveParamSpec } from "./passives.js";

/** Kinds this module compiles (signal_gen). */
export const SIGNAL_GEN_KINDS: ReadonlySet<string> = new Set(["signal_gen"]);

export function compileSignalGenKind(ctx: KindCompileContext, component: SimComponent, componentIndex: number): void {
  void componentIndex;
  declineUnsupportedKind(ctx, component);
}

/** Live-param specs of these kinds, folded into `LIVE_PARAM_SPECS` (compile/index.ts). */
export const SIGNAL_GEN_LIVE_PARAM_SPECS: ReadonlyMap<string, LiveParamSpec> = new Map();
